import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.spbunited.robot import control_pb2
from multiprocessing.shared_memory import SharedMemory
import struct
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((config.BIND_HOST, config.CMD_PORT))
while True:
    data, addr = sock.recvfrom(4096)
    print(data)
    cmd = control_pb2.OldFormat()
    cmd.ParseFromString(data)

    json_str = MessageToJson(cmd)
    json_bytes = json_str.encode('utf-8')
    print(json_str)
