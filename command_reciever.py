import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.spbunited.robot import control_pb2
from multiprocessing.shared_memory import SharedMemory
import struct
import time
import json

def command_reciever():
    shm = SharedMemory(name=config.SHM_NAME)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.BIND_HOST, config.CMD_PORT))

    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)

    while True:
        # print("A"*32)
        raw_bytes = shm.buf[:4]
        cx, cy = struct.unpack("HH", raw_bytes)#Ye

        if cx == 1281 and cy == 961:
            print(f"Мяч не найден")
        else:
            print(f"Координаты мяча: X={cx}, Y={cy}")
        # time.sleep(0.1)
        # data, addr = sock.recvfrom(4096)

        # cmd = control_pb2.OldFormat()
        # cmd.ParseFromString(data)

        json_str = json.dumps({
        "xvel": 0.5,
        "yvel": 0.0,
        "wvel": 0.0,
        "dribbler": 0,
        "voltage": 0,
        "kick_lower": False,
        "kick_upper": False,
        "autokick_lower": False,
        "autokick_upper": False,
        "autokick_momentum": False,
        "angle_mode": False
        })
        json_bytes = json_str.encode('utf-8')
        print(json_bytes)
        ser.write(json_bytes)


# {
#   "xvel": 0.0,
#   "yvel": 0.0,
#   "wvel": 0.0,
#   "dribbler": 0,
#   "voltage": 0,
#   "kick_lower": false,
#   "kick_upper": false,
#   "autokick_lower": false,
#   "autokick_upper": false,
#   "autokick_momentum": false,
#   "angle_mode": false
# }

