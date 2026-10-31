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


class TelemetryTransceiver:
    """Forwards telemetry frames from the MCU as UDP broadcast protobuf packets.

    Also agrees the robot id with the MCU. Both send their id in every frame; the MCU takes ours
    right away unless its id was just changed with the buttons, and we take the MCU's once it has
    differed for ROBOT_ID_ADOPT_S, renaming the RPi to fb4-NN (the hostname is where the id is kept).
    """

    def __init__(
        self,
        ser,
        sock: socket.socket,
        ball_detector: BallDetector,
        robot_id=None,
        dest: tuple[str, int] = ("<broadcast>", config.TEL_PORT),
        read_rpi_temp: Callable[[], float] = get_rpi_temp_c,
        save_id: Callable[[int], None] = rename_rpi,
        clock: Callable[[], float] = monotonic,
    ):
        self.ser = ser
        self.sock = sock
        self.ball_detector = ball_detector
        # multiprocessing.Value shared with command_receiver, which sends it to the MCU
        if robot_id is None:
            robot_id = multiprocessing.Value("i", config.ROBOT_ID)
        self.robot_id = robot_id
        self.dest = dest
        self.read_rpi_temp = read_rpi_temp
        self.save_id = save_id
        self.clock = clock
        self.parser = FrameParser(TEL_FMT)
        self.last_rpi_status = 0.01
        self.id_mismatch: Optional[tuple[int, float]] = None  # (MCU id, since)
        self.send_failing = False

    def sync_robot_id(self, mcu_id: int) -> None:
        if mcu_id == self.robot_id.value or mcu_id > config.MAX_ROBOT_ID:
            self.id_mismatch = None
            return
        now = self.clock()
        if self.id_mismatch is None or self.id_mismatch[0] != mcu_id:
            self.id_mismatch = (mcu_id, now)
        if now - self.id_mismatch[1] < config.ROBOT_ID_ADOPT_S:
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
            packet = build_telemetry_packet(self.robot_id.value, tel, self.ball_detector.r_pos, rpi_temp)
            self.send(packet.SerializeToString())
        return len(frames)

    def send(self, data: bytes) -> None:
        # The runner starts before Wi-Fi is up: until then there is no route for the broadcast
        try:
            self.sock.sendto(data, self.dest)
        except OSError as e:
            if not self.send_failing:
                print(f"can't send telemetry yet ({e}), dropping it until the network is up")
            self.send_failing = True
            return
        self.send_failing = False


def telemetry_transceiver(ball_detector: BallDetector, robot_id):
    if config.hostname_robot_id() is None:  # the strategy finds the robot by fb4-NN.local
        rename_rpi(robot_id.value)
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    transceiver = TelemetryTransceiver(ser, sock, ball_detector, robot_id)
    while True:
        transceiver.step()
