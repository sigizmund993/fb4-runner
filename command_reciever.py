import socket
import serial
import config
from google.protobuf.json_format import MessageToJson
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from multiprocessing.shared_memory import SharedMemory
import struct
from utils.uart_utils import send_command
from to_real_coords.coord_converter import CoordConverter
def command_reciever():
    # shm = SharedMemory(name=config.SHM_NAME)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.BIND_HOST, config.CMD_PORT))
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    # coord_converter = CoordConverter("to_real_coords/camera_params_fisheye.npz","to_real_coords/homography_matrix.npy")
    while True:
        # raw_bytes = shm.buf[:4]
        # cx, cy = struct.unpack("HH", raw_bytes)#Ye
        # if cx == 1281 and cy == 961:
        #     print(f"Мяч не найден")
        # else:
        #     real_coords = coord_converter.get_coords(cx,cy)
        #     print("real coords: ",real_coords)
        # time.sleep(0.1)
        data, addr = sock.recvfrom(4096)
        # print("z")
        # print(data)
        cmd = control_pb2.NewFormat()
        cmd.ParseFromString(data)
        new_format = cmd#.new_format
        # print(cmd)

        if new_format.speed_control.angular_velocity:
            w = new_format.speed_control.angular_velocity
            angle_mode = False
        elif new_format.speed_control.delta_angle:
            w = new_format.speed_control.delta_angle
            angle_mode = True
        else:
            w = 0
            angle_mode = True
        print(new_format.speed_control.vel_y, -new_format.speed_control.vel_x, -w,
                                 new_format.kicker_and_dribbler.dribbler_setting, new_format.kicker_and_dribbler.kicker_setting,
                                 new_format.kicker_and_dribbler.kicker_mode, angle_mode)

        ser.write(send_command(new_format.speed_control.vel_y, -new_format.speed_control.vel_x, -w,
                                 new_format.kicker_and_dribbler.dribbler_setting, new_format.kicker_and_dribbler.kicker_setting,
                                 new_format.kicker_and_dribbler.kicker_mode, angle_mode))
