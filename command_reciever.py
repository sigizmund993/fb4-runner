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
        # raw_bytes = shm.buf[:4]
        # cx, cy = struct.unpack("HH", raw_bytes)#Ye
        # if cx == 1281 and cy == 961:
        #     print(f"Мяч не найден")
        # else:
        #     real_coords = coord_converter.get_coords(cx,cy)
        #     print("real coords: ",real_coords)
        time.sleep(0.1)
        data, addr = sock.recvfrom(4096)

        cmd = control_pb2.RobotCommand()
        cmd.ParseFromString(data)
        if cmd.new_format:
            new_format = cmd.new_format
            if new_format.speed_control.angular_velocity:
                w = new_format.speed_control.angular_velocity
                angle_mode = False
            elif new_format.speed_control.delta_angle:
                w = new_format.speed_control.delta_angle
                angle_mode = True
            else:
                w = 0
                angle_mode = True
            json_str = json.dumps({
            "xvel": new_format.speed_control.vel_x,
            "yvel": new_format.speed_control.vel_y,
            "wvel": w,
            "dribbler": new_format.kicker_and_dribbler.dribbler_setting,
            "voltage": new_format.kicker_and_dribbler.kicker_setting,
            "kick_lower": new_format.kicker_and_dribbler.kicker_mode,
            "kick_upper": False,
            "autokick_lower": False,
            "autokick_upper": False,
            "autokick_momentum": False,
            "angle_mode": angle_mode
            })
            # print(old_format.vel_x,old_format.vel_y,old_format.angular_velocity_or_delta_angle)
            json_bytes = json_str.encode('utf-8')
            # print(json_bytes)
            ser.write(json_bytes)
            ser.write(b"\n")