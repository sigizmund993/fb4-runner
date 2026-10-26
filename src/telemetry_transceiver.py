import socket
from time import time
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


class TelemetryTransceiver:
    """Forwards telemetry frames from the MCU as UDP broadcast protobuf packets."""

    def __init__(
        self,
        ser,
        sock: socket.socket,
        ball_detector: BallDetector,
        robot_id: int = config.ROBOT_ID,
        dest: tuple[str, int] = ("<broadcast>", config.TEL_PORT),
        read_rpi_temp: Callable[[], float] = get_rpi_temp_c,
    ):
        self.ser = ser
        self.sock = sock
        self.ball_detector = ball_detector
        self.robot_id = robot_id
        self.dest = dest
        self.read_rpi_temp = read_rpi_temp
        self.parser = FrameParser(TEL_FMT)
        self.last_rpi_status = 0.0

    def step(self) -> int:
        """Reads pending UART bytes and sends a packet per frame. Returns the number sent."""
        # Blocks for up to the serial timeout when nothing is pending
        frames = self.parser.feed(self.ser.read(self.ser.in_waiting or 1))
        if not frames:
            return 0
        self.ball_detector.update()

        for frame in frames:
            rpi_temp = None
            if time() - self.last_rpi_status > RPI_STATUS_PERIOD_S:
                rpi_temp = self.read_rpi_temp()
                self.last_rpi_status = time()
            packet = build_telemetry_packet(self.robot_id, Telemetry(*frame), self.ball_detector.r_pos, rpi_temp)
            self.sock.sendto(packet.SerializeToString(), self.dest)
        return len(frames)


def telemetry_transceiver(ball_detector: BallDetector):
    change_hostname(f"fb4-{config.ROBOT_ID:02d}.local")
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    transceiver = TelemetryTransceiver(ser, sock, ball_detector)
    while True:
        transceiver.step()
