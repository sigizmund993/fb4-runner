"""Telemetry path: UART frames from the STM32 -> UDP protobuf broadcast."""
import multiprocessing
import select
import socket
import struct
import unittest

import helpers

helpers.require_protobuf()

import config  # noqa: E402
from protocol.uart import TEL_FMT, Telemetry, make_frame  # noqa: E402
from ssl_packet_package.protopy.spbunited.robot import telemetry_pb2  # noqa: E402
from telemetry_transceiver import TelemetryTransceiver, build_telemetry_packet  # noqa: E402
from utils.auxiliary import aux  # noqa: E402

DEST = ("127.0.0.1", 5556)


def mcu_frame(robot_id=5, voltage=12.5, ball_deep=False, ball_front=False) -> bytes:
    """Builds a telemetry frame the way the firmware does (UartControl::send_telemetry)."""
    return make_frame(struct.pack(TEL_FMT, robot_id, voltage, ball_deep, ball_front))


class FakeSerial:
    """Returns the queued chunks one per read(), then nothing."""

    def __init__(self, *chunks: bytes):
        self.chunks = list(chunks)

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size: int = 1) -> bytes:
        return self.chunks.pop(0) if self.chunks else b""


class FakeSocket:
    def __init__(self):
        self.sent: list[tuple[bytes, tuple]] = []

    def sendto(self, data: bytes, addr: tuple) -> None:
        self.sent.append((data, addr))

    def packets(self) -> list[telemetry_pb2.RobotTelemetry]:
        return [telemetry_pb2.RobotTelemetry.FromString(data) for data, _ in self.sent]


class FakeBallDetector:
    def __init__(self, r_pos=None):
        self.r_pos = r_pos
        self.updates = 0

    def update(self):
        self.updates += 1


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def make_transceiver(ser, sock=None, ball_detector=None, rpi_temp=48.5, robot_id=7, renames=None, clock=None):
    return TelemetryTransceiver(
        ser,
        sock if sock is not None else FakeSocket(),
        ball_detector if ball_detector is not None else FakeBallDetector(),
        robot_id=multiprocessing.Value("i", robot_id),
        dest=DEST,
        read_rpi_temp=lambda: rpi_temp,
        save_id=(renames if renames is not None else []).append,
        clock=clock if clock is not None else FakeClock(),
    )


class BuildTelemetryPacketTest(unittest.TestCase):
    def test_strategy_fields(self):
        packet = build_telemetry_packet(5, Telemetry(5, 12.5, True, False), None, None)
        self.assertEqual(packet.robot_id, 5)
        st = packet.strategy_telemetry
        self.assertEqual((st.ball_deep, st.ball_in, st.kicker_voltage), (True, False, 12.5))
        self.assertFalse(st.HasField("ball_position"))
        self.assertFalse(packet.HasField("raspberry_status"))

    def test_ball_position_and_rpi_temp(self):
        packet = build_telemetry_packet(5, Telemetry(5, 0.0, False, True), aux.Point(0.25, -1.5), 47.0)
        self.assertEqual(
            (packet.strategy_telemetry.ball_position.x, packet.strategy_telemetry.ball_position.y), (0.25, -1.5)
        )
        self.assertEqual(packet.raspberry_status.rpi_temp, 47.0)

    def test_packet_serializes(self):
        # Required proto2 fields must be set, otherwise SerializeToString raises
        packet = build_telemetry_packet(5, Telemetry(5, 12.5, False, False), None, None)
        self.assertTrue(packet.SerializeToString())


class TelemetryTransceiverTest(unittest.TestCase):
    def test_frame_is_forwarded(self):
        sock = FakeSocket()
        tx = make_transceiver(FakeSerial(mcu_frame(robot_id=7, voltage=20.0, ball_deep=True, ball_front=True)), sock)
        self.assertEqual(tx.step(), 1)
        self.assertEqual(sock.sent[0][1], DEST)
        self.assertEqual(sock.packets()[0].robot_id, 7)
        st = sock.packets()[0].strategy_telemetry
        self.assertEqual((st.kicker_voltage, st.ball_deep, st.ball_in), (20.0, True, True))

    def test_multiple_frames_with_noise_in_one_read(self):
        sock = FakeSocket()
        data = b"\x00\x13" + mcu_frame(voltage=1.0) + b"\xa5junk" + mcu_frame(voltage=2.0)
        tx = make_transceiver(FakeSerial(data), sock)
        self.assertEqual(tx.step(), 2)
        self.assertEqual([p.strategy_telemetry.kicker_voltage for p in sock.packets()], [1.0, 2.0])

    def test_corrupted_frame_is_dropped(self):
        sock = FakeSocket()
        bad = bytearray(mcu_frame(voltage=1.0))
        bad[3] ^= 0x01
        tx = make_transceiver(FakeSerial(bytes(bad) + mcu_frame(voltage=2.0)), sock)
        self.assertEqual(tx.step(), 1)
        self.assertEqual(sock.packets()[0].strategy_telemetry.kicker_voltage, 2.0)

    def test_frame_split_across_reads(self):
        sock = FakeSocket()
        ball = FakeBallDetector()
        frame = mcu_frame(voltage=3.0)
        tx = make_transceiver(FakeSerial(frame[:4], frame[4:]), sock, ball)
        self.assertEqual(tx.step(), 0)
        self.assertEqual((sock.sent, ball.updates), ([], 0))
        self.assertEqual(tx.step(), 1)
        self.assertEqual(sock.packets()[0].strategy_telemetry.kicker_voltage, 3.0)

    def test_no_data(self):
        sock = FakeSocket()
        tx = make_transceiver(FakeSerial(), sock)
        self.assertEqual(tx.step(), 0)
        self.assertEqual(sock.sent, [])

    def test_ball_position_from_camera(self):
        sock = FakeSocket()
        ball = FakeBallDetector(aux.Point(0.5, 0.75))
        tx = make_transceiver(FakeSerial(mcu_frame()), sock, ball)
        tx.step()
        self.assertEqual(ball.updates, 1)
        pos = sock.packets()[0].strategy_telemetry.ball_position
        self.assertEqual((pos.x, pos.y), (0.5, 0.75))

    def test_no_ball_position_when_not_seen(self):
        sock = FakeSocket()
        tx = make_transceiver(FakeSerial(mcu_frame()), sock, FakeBallDetector(None))
        tx.step()
        self.assertFalse(sock.packets()[0].strategy_telemetry.HasField("ball_position"))

    def test_rpi_temp_is_rate_limited(self):
        sock = FakeSocket()
        tx = make_transceiver(FakeSerial(mcu_frame(), mcu_frame()), sock, rpi_temp=51.0)
        tx.step()
        tx.step()
        first, second = sock.packets()
        self.assertEqual(first.raspberry_status.rpi_temp, 51.0)
        self.assertFalse(second.HasField("raspberry_status"))

    def test_same_robot_id_keeps_hostname(self):
        renames = []
        tx = make_transceiver(FakeSerial(*[mcu_frame(robot_id=7)] * 3), renames=renames)
        while tx.step():
            pass
        self.assertEqual((tx.robot_id.value, renames), (7, []))

    def test_mcu_id_taken_once_it_lasts(self):
        # The MCU's id was changed with the buttons: it keeps it, so the difference lasts
        renames, clock, sock = [], FakeClock(), FakeSocket()
        tx = make_transceiver(FakeSerial(*[mcu_frame(robot_id=3)] * 3), sock, renames=renames, clock=clock)
        tx.step()
        clock.now += config.ROBOT_ID_ADOPT_S * 0.9
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (7, []), "the MCU gets our id first")
        clock.now += config.ROBOT_ID_ADOPT_S * 0.2
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (3, [3]))
        self.assertEqual([p.robot_id for p in sock.packets()], [7, 7, 3])

    def test_mcu_taking_our_id_is_not_a_change(self):
        # On connect the MCU sends its old id until it gets ours
        renames, clock = [], FakeClock()
        tx = make_transceiver(FakeSerial(mcu_frame(robot_id=3), mcu_frame(robot_id=7)), renames=renames, clock=clock)
        tx.step()
        clock.now += config.ROBOT_ID_ADOPT_S * 0.5
        tx.step()
        clock.now += config.ROBOT_ID_ADOPT_S
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (7, []))

    def test_mcu_id_change_restarts_the_wait(self):
        # Clicking through ids with the buttons renames the RPi once, to the final id
        renames, clock = [], FakeClock()
        tx = make_transceiver(FakeSerial(*[mcu_frame(robot_id=i) for i in (8, 9, 9)]), renames=renames, clock=clock)
        tx.step()
        clock.now += config.ROBOT_ID_ADOPT_S * 0.8
        tx.step()
        clock.now += config.ROBOT_ID_ADOPT_S * 0.8
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (7, []))
        tx.ser.chunks.append(mcu_frame(robot_id=9))
        clock.now += config.ROBOT_ID_ADOPT_S * 0.3
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (9, [9]))

    def test_unknown_id_takes_the_mcus_at_once(self):
        renames, sock = [], FakeSocket()
        tx = make_transceiver(FakeSerial(mcu_frame(robot_id=4)), sock, robot_id=-1, renames=renames)
        tx.step()
        self.assertEqual((tx.robot_id.value, renames, sock.packets()[0].robot_id), (4, [4], 4))

    def test_invalid_mcu_id_is_ignored(self):
        renames = []
        tx = make_transceiver(FakeSerial(mcu_frame(robot_id=200)), robot_id=-1, renames=renames)
        tx.step()
        self.assertEqual((tx.robot_id.value, renames), (-1, []))

    def test_real_udp_delivery(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.bind(("127.0.0.1", 0))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            tx = TelemetryTransceiver(
                FakeSerial(mcu_frame(voltage=9.5, ball_front=True)),
                sender,
                FakeBallDetector(),
                dest=listener.getsockname(),
                read_rpi_temp=lambda: 40.0,
            )
            self.assertEqual(tx.step(), 1)
            ready, _, _ = select.select([listener], [], [], 1.0)
            self.assertTrue(ready, "telemetry datagram not received")
            packet = telemetry_pb2.RobotTelemetry.FromString(listener.recv(1024))
            self.assertEqual((packet.strategy_telemetry.kicker_voltage, packet.strategy_telemetry.ball_in), (9.5, True))
        finally:
            listener.close()
            sender.close()


if __name__ == "__main__":
    unittest.main()
