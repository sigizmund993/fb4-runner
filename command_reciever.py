import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from multiprocessing.shared_memory import SharedMemory
import struct
from utils.uart_utils import send_command
from to_real_coords.coord_converter import CoordConverter
from typing import Optional
from utils import aux
from time import time
def command_reciever():
    shm = SharedMemory(name=config.SHM_NAME)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.BIND_HOST, config.CMD_PORT))
    sock.setblocking(False)
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    new_format = None
    ball_x:Optional[int] = None
    ball_y:Optional[int] = None
    last_update = time()
    # coord_converter = CoordConverter("to_real_coords/camera_params_fisheye.npz","to_real_coords/homography_matrix.npy")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            # print("z")
            # print(data)
            cmd = control_pb2.NewFormat()
            cmd.ParseFromString(data)
            new_format = cmd#.new_format
        except BlockingIOError:
            pass
        raw_bytes = shm.buf[:4]
        cx, cy = struct.unpack("HH", raw_bytes)#Ye
        if cx == 1281 and cy == 961:
            ball_x = None
            ball_y = None
        else:
            # real_coords = coord_converter.get_coords(cx,cy)
            ball_x = 640-cx
            ball_y = 480-cy
        # print(ball_x,ball_y)
        if new_format:
            if new_format.speed_control.angular_velocity:
                w = new_format.speed_control.angular_velocity
                angle_mode = False
            elif new_format.speed_control.delta_angle:
                w = new_format.speed_control.delta_angle
                angle_mode = True
            else:
                w = 0
                angle_mode = True
            vel_x = -new_format.speed_control.vel_x
            vel_y = new_format.speed_control.vel_y
            if ball_x is not None and new_format.kicker_and_dribbler.kicker_mode == 1:
                err = ball_x/640
                vel_x = err*1 #+ (err-prev_err)*0.1
                prev_err = err
                
                # aux.minmax(vel_x,1)
            # print(vel_y,vel_x , -w,
            #                         new_format.kicker_and_dribbler.dribbler_setting, new_format.kicker_and_dribbler.kicker_setting,
            #                         new_format.kicker_and_dribbler.kicker_mode, angle_mode)

            ser.write(send_command(vel_y,vel_x, -w,
                                    new_format.kicker_and_dribbler.dribbler_setting, new_format.kicker_and_dribbler.kicker_setting,
                                    new_format.kicker_and_dribbler.kicker_mode, angle_mode))        
        else:
            pass
            # print("no new command")
        print(time()-last_update)
        last_update = time()  
