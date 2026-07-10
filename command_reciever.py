import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from multiprocessing.shared_memory import SharedMemory
import struct
import time
import json
from to_real_coords.coord_converter import CoordConverter
def command_reciever():
    shm = SharedMemory(name=config.SHM_NAME)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.BIND_HOST, config.CMD_PORT))
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    coord_converter = CoordConverter("to_real_coords/camera_params_fisheye.npz","to_real_coords/homography_matrix.npy")
    while True:
        raw_bytes = shm.buf[:4]
        cx, cy = struct.unpack("HH", raw_bytes)#Ye
        if cx == 1281 and cy == 961:
            print(f"Мяч не найден")
        else:
            real_coords = coord_converter.get_coords(cx,cy)
            print("real coords: ",real_coords)
        time.sleep(0.1)
        data, addr = sock.recvfrom(4096)

        cmd = control_pb2.RobotCommand()
        cmd.ParseFromString(data)
        if cmd.old_format:
            old_format = cmd.old_format
            json_str = json.dumps({
            "xvel": old_format.vel_x,
            "yvel": old_format.vel_y,
            "wvel": old_format.angular_velocity_or_delta_angle,
            "dribbler": old_format.dribbler_setting,
            "voltage": old_format.kicker_setting,
            "kick_lower": old_format.kick_straight,
            "kick_upper": old_format.kick_high,
            "autokick_lower": old_format.autokick_straight,
            "autokick_upper": old_format.autokick_high,
            "autokick_momentum": False,
            "angle_mode": old_format.angvel_angle_toggle
            })
            print(json_str)
            json_bytes = json_str.encode('utf-8')
            # print(json_bytes)
            ser.write(json_bytes)