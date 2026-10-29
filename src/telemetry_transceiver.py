import multiprocessing
import socket
import subprocess
from time import monotonic, time
from typing import Callable, Optional

import serial

import config
from ball_detector import BallDetector
from protocol.uart import TEL_FMT, FrameParser, Telemetry
from ssl_packet_package.protopy.spbunited.robot import telemetry_pb2
from utils.auxiliary import aux
from utils import envfile
from utils.change_hostname import change_hostname

RPI_STATUS_PERIOD_S = 1.0


def get_rpi_temp_c():
    path = "/sys/class/thermal/thermal_zone0/temp"
    with open(path, "r") as f:
        temp_millideg = int(f.read().strip())
    return temp_millideg / 1000.0


def build_telemetry_packet(
    robot_id: int, tel: Telemetry, ball_pos: Optional[aux.Point], rpi_temp: Optional[float]
) -> telemetry_pb2.RobotTelemetry:
    packet = telemetry_pb2.RobotTelemetry()
    packet.robot_id = robot_id
    packet.strategy_telemetry.ball_deep = tel.ball_deep
    packet.strategy_telemetry.ball_in = tel.ball_front
    packet.strategy_telemetry.kicker_voltage = tel.voltage
    if ball_pos is not None:
        packet.strategy_telemetry.ball_position.x = ball_pos.x
        packet.strategy_telemetry.ball_position.y = ball_pos.y
    if rpi_temp is not None:
        packet.raspberry_status.rpi_temp = rpi_temp
    return packet


def rename_rpi(robot_id: int) -> None:
    try:
        change_hostname(config.robot_hostname(robot_id))
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"can't change hostname: {e}")


def save_robot_id(robot_id: int) -> None:
    """Keeps the id over restarts: otherwise .env would bring back the old one and the STM32 would take it."""
    try:
        envfile.put(config.ROBOT_ID_KEY, str(robot_id))
    except OSError as e:
        print(f"can't save robot id to {envfile.ENV_FILE}: {e}")
    rename_rpi(robot_id)


class TelemetryTransceiver:
    """Forwards telemetry frames from the MCU as UDP broadcast protobuf packets.

    Also agrees the robot id with the MCU. Both send their id in every frame; the MCU takes ours
    right away unless its id was just changed with the buttons, and we take the MCU's once it has
    differed for ROBOT_ID_ADOPT_S, saving it to .env and renaming the RPi to fb4-NN.
    """

    def __init__(
        self,
        ser,
        sock: socket.socket,
        ball_detector: BallDetector,
        robot_id=None,
        dest: tuple[str, int] = ("<broadcast>", config.TEL_PORT),
        read_rpi_temp: Callable[[], float] = get_rpi_temp_c,
        save_id: Callable[[int], None] = save_robot_id,
        clock: Callable[[], float] = monotonic,
    ):
        self.ser = ser
        self.sock = sock
        self.ball_detector = ball_detector
        # multiprocessing.Value shared with command_receiver, which sends it to the MCU; -1 = unknown
        if robot_id is None:
            robot_id = multiprocessing.Value("i", -1 if config.ROBOT_ID is None else config.ROBOT_ID)
        self.robot_id = robot_id
        self.dest = dest
        self.read_rpi_temp = read_rpi_temp
        self.save_id = save_id
        self.clock = clock
        self.parser = FrameParser(TEL_FMT)
        self.last_rpi_status = 0.01
        self.id_mismatch: Optional[tuple[int, float]] = None  # (MCU id, since)

    def sync_robot_id(self, mcu_id: int) -> None:
        if mcu_id == self.robot_id.value or mcu_id > config.MAX_ROBOT_ID:
            self.id_mismatch = None
            return
        now = self.clock()
        if self.id_mismatch is None or self.id_mismatch[0] != mcu_id:
            self.id_mismatch = (mcu_id, now)
        if self.robot_id.value >= 0 and now - self.id_mismatch[1] < config.ROBOT_ID_ADOPT_S:
            return
        print(f"robot id {self.robot_id.value} -> {mcu_id} from the MCU")
        self.robot_id.value = mcu_id
        self.id_mismatch = None
        self.save_id(mcu_id)

    def step(self) -> int:
        """Reads pending UART bytes and sends a packet per frame. Returns the number sent."""
        # Blocks for up to the serial timeout when nothing is pending
        frames = self.parser.feed(self.ser.read(self.ser.in_waiting or 1))
        if not frames:
            return 0
        self.ball_detector.update()

        for frame in frames:
            tel = Telemetry(*frame)
            self.sync_robot_id(tel.robot_id)
            rpi_temp = None
            if time() - self.last_rpi_status > RPI_STATUS_PERIOD_S:
                rpi_temp = self.read_rpi_temp()
                self.last_rpi_status = time()
            robot_id = self.robot_id.value if self.robot_id.value >= 0 else tel.robot_id
            packet = build_telemetry_packet(robot_id, tel, self.ball_detector.r_pos, rpi_temp)
            self.sock.sendto(packet.SerializeToString(), self.dest)
        return len(frames)


def telemetry_transceiver(ball_detector: BallDetector, robot_id):
    if robot_id.value >= 0 and config.hostname_robot_id() != robot_id.value:  # FB4_ROBOT_ID given
        rename_rpi(robot_id.value)
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    transceiver = TelemetryTransceiver(ser, sock, ball_detector, robot_id)
    while True:
        transceiver.step()
