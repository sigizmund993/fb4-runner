import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.robot import control_pb2


def command_sender(shared_ball_pos):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.BIND_HOST, config.CMD_PORT))

    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)

    while True:
        data, addr = sock.recvfrom(4096)

        cmd = control_pb2.OldFormat()
        cmd.ParseFromString(data)

        json_str = MessageToJson(cmd)
        json_bytes = json_str.encode('utf-8')

        ser.write(json_bytes)
