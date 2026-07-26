import socket
import serial
import config
import struct
from utils.uart_utils import compute_checksum,SYNC_BYTE,PKT_LEN
from utils.change_hostname import change_hostname
from ssl_packet_package.protopy.spbunited.robot import telemetry_pb2
def telemetry_tranciever():    
    # while True:
    #     pass
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    cur_robot_id = 0
    change_hostname(f"fb4-{cur_robot_id:02d}.local")
    _rx_buf = bytearray()

    def read_telemetry():
        # read whatever is available
        waiting = ser.in_waiting
        if waiting > 0:
            _rx_buf.extend(ser.read(waiting))

        # scan for sync byte
        while len(_rx_buf) >= PKT_LEN:
            # find sync byte
            idx = _rx_buf.find(SYNC_BYTE)
            if idx < 0:
                _rx_buf.clear()
                return None
            if idx > 0:
                del _rx_buf[:idx]  # discard bytes before sync

            if len(_rx_buf) < PKT_LEN:
                return None

            pkt = bytes(_rx_buf[:PKT_LEN])
            if compute_checksum(pkt[:PKT_LEN - 1]) != pkt[PKT_LEN - 1]:
                del _rx_buf[0]  # bad checksum, skip this sync byte
                continue

            del _rx_buf[:PKT_LEN]
            _, robot_id, voltage, ball_deep, ball_front, _ = struct.unpack('<BBe??B', pkt)
            return {
                'robot_id': robot_id,
                'voltage': voltage,
                'ball_deep': ball_deep,
                'ball_front': ball_front,
            }

        return None
    while True:
        packet = read_telemetry()
        if packet:
            # print(packet)
            if packet["robot_id"] != cur_robot_id:
                cur_robot_id = packet["robot_id"]
                change_hostname(f"fb4-{cur_robot_id:02d}.local")
            proto_package = telemetry_pb2.RobotTelemetry()
            proto_package.strategy_telemetry.ball_deep = packet["ball_deep"]
            proto_package.strategy_telemetry.ball_in = packet["ball_front"]
            proto_package.strategy_telemetry.kicker_voltage = packet["voltage"]
            sock.sendto(proto_package.SerializeToString(), ("<broadcast>", config.TEL_PORT))
            