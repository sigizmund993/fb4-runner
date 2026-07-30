import socket
import serial
import config
import attr
import math
from ssl_packet_package.protopy.spbunited.robot import control_pb2
import struct
from utils.auxiliary import aux
from time import time,sleep
from vision_reciever import Team
from vision_reciever import VisionClient
from vision_reciever import Geometry
from ball_detector import BallDetector
import struct
SYNC_BYTE = 0xA5
PKT_LEN = 7
def compute_checksum(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc
class Calman:
    def __init__(self,alpha:float,beta:float,x:float,is_angle:bool = False)->None:
        self.alpha = alpha
        self.beta = beta
        self.x = x
        self.dx = 0
        self.is_angle = is_angle
        self.last_update = time()
    def update(self,new_x:float):
        dt = time()-self.last_update
        self.last_update = time()
        pred_x = self.x+self.dx * dt
        if self.is_angle:
            res_x = math.atan2(math.sin(new_x - pred_x), math.cos(new_x - pred_x))
        else:
            res_x = new_x - pred_x
        self.x = pred_x + self.alpha *res_x
        self.dx = self.dx + (self.beta/dt)*res_x
class Ball:
    def __init__(self,pos:aux.Point):
        self.pos = pos
        self.vel = aux.Point(0,0)
        self.calman_x = Calman(0.6,0.15,self.pos.x)
        self.calman_y = Calman(0.6,0.15,self.pos.y)

    def update(self,pos:aux.Point):
        self.calman_x.update(pos.x)
        self.calman_y.update(pos.y)

        self.vel = aux.Point(self.calman_x.dx,self.calman_y.dx)
        self.pos = aux.Point(self.calman_x.x,self.calman_y.x)

class Robot:
    def __init__(self,pos:aux.Point,ang:float,r_id:int,team:Team)->None:
        self.pos = pos
        self.ang = ang
        self.id = r_id
        self.team = team
        self.vel = aux.Point(0,0)
        self.angle_vel = 0.0
        self.calman_x = Calman(0.6,0.15,self.pos.x)
        self.calman_y = Calman(0.6,0.15,self.pos.y)
        self.calman_ang = Calman(0.6,0.15,self.ang,True)
    def update(self,pos:aux.Point,ang:float)->None:
        self.calman_x.update(pos.x)
        self.calman_y.update(pos.y)
        self.calman_ang.update(ang)
        self.vel = aux.Point(self.calman_x.dx,self.calman_y.dx)
        self.pos = aux.Point(self.calman_x.x,self.calman_y.x)
        self.angle_vel = self.calman_ang.dx
        self.ang = self.calman_ang.x
        self.last_update = time()
class Field:
    def __init__(self,client:VisionClient)->None:
        self.client = client
        self.ball = Ball(aux.Point(0,0))
        self.yellow_robots = [Robot(aux.Point(0,0),0,i,Team.YELLOW)for i in range(16)]
        self.blue_robots = [Robot(aux.Point(0,0),0,i,Team.BLUE)for i in range(16)]
        self.geometry = Geometry(0,0,0,0,0,0,0,0)
        sleep(1)
        self.update()
    def update(self):
        detection = self.client.get_detection()
        if len(detection.balls)>0:
            self.ball.update(detection.balls[0].pos/1000)
        for rbt_det in detection.robots:
            if rbt_det.team == Team.YELLOW:
                self.yellow_robots[rbt_det.robot_id].update(rbt_det.pos/1000,rbt_det.orientation)
            if rbt_det.team == Team.BLUE:
                self.blue_robots[rbt_det.robot_id].update(rbt_det.pos/1000,rbt_det.orientation)
        if detection.geometry:
            self.geometry = detection.geometry


@attr.s(auto_attribs=True)
class UartCommand:
    vel:aux.Point = aux.Point(0,0)
    w:float = 0
    dribbler_speed:float = 0
    kicker_voltage:float = 0
    kicker_mode:int = 0
    angle_mode:bool = True#False - angle vel, true - delta angle
    def to_bytes(self):
        header = struct.pack('<B5eB?',
                                 SYNC_BYTE,
                                 self.vel.x, self.vel.y, self.w,
                                 self.dribbler_speed, self.kicker_voltage,
                                 self.kicker_mode, self.angle_mode)
        checksum = compute_checksum(header)
        packet = header + struct.pack('B', checksum)
        return packet
    
class CommandReciever:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((config.BIND_HOST, config.CMD_PORT))
        self.sock.setblocking(False)
        self.last_cmd = control_pb2.RobotCommand()
    def update(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            cmd = control_pb2.RobotCommand()
            cmd.ParseFromString(data)
            self.last_cmd = cmd
        except BlockingIOError:
            pass
    

def command_reciever(client: VisionClient,ball_detector:BallDetector):
    field = Field(client)
    command_reciever = CommandReciever()
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    robot = field.yellow_robots[5]
    def parse_spdc(spdc:control_pb2.SpeedControl,uart_command:UartCommand):
        uart_command.vel = aux.Point(spdc.vel_x,spdc.vel_y)
        if spdc.angular_velocity:
            uart_command.w = spdc.angular_velocity
            uart_command.angle_mode = False
        elif spdc.delta_angle:
            uart_command.w = spdc.delta_angle
            uart_command.angle_mode = True
        elif spdc.global_angle:
            pass
            # uart_command.angle_mode = True
            # err = math.atan2(math.sin(spdc.global_angle - my_angle), math.cos(spdc.global_angle - my_angle))
        else:
            uart_command.w = 0
            uart_command.angle_mode = True
    def parse_kad(kad:control_pb2.KickerAndDribbler,uart_command:UartCommand):
        uart_command.dribbler_speed = kad.dribbler_setting
        uart_command.kicker_voltage = kad.kicker_setting
        uart_command.kicker_mode = kad.kicker_mode
    def go_to_point_ignore(tgt_pos:aux.Point,tgt_ang:float,uart_command:UartCommand):
        rbt_pos = robot.pos
        err = tgt_pos-rbt_pos
        glob_vel = err * 1
        uart_command.vel = aux.rotate(glob_vel,-robot.ang)
        uart_command.w = aux.wind_down_angle(tgt_ang-robot.ang)
        uart_command.angle_mode = True
    while True:
        ball_detector.update()
        field.update()
        command_reciever.update()
        last_cmd = command_reciever.last_cmd
        uart_command = UartCommand()
        match last_cmd.RobotControlType:
            case control_pb2.RobotControlType.NEW_FORMAT:
                parse_spdc(last_cmd.new_format.speed_control,uart_command)                
                parse_kad(last_cmd.new_format.kicker_and_dribbler,uart_command)
            case control_pb2.RobotControlType.KICKER_AND_DRIBBLER:
                parse_kad(last_cmd.kicker_and_dribbler,uart_command)
            case control_pb2.RobotControlType.SPEED_CONTROL:
                parse_spdc(last_cmd.speed_control,uart_command)
            case control_pb2.RobotControlType.OLD_FORMAT:
                cmd = last_cmd.old_format
                uart_command.vel = aux.Point(cmd.vel_x,cmd.vel_y)
                uart_command.w = cmd.angular_velocity_or_delta_angle
                uart_command.kicker_voltage = cmd.kicker_setting
                uart_command.dribbler_speed = cmd.dribbler_setting * cmd.dribbler_is_enabled
                uart_command.angle_mode = cmd.angvel_angle_toggle
                if cmd.kick_straight:
                    uart_command.kicker_mode = 1
                elif cmd.kick_high:
                    uart_command.kicker_mode = 2
                elif cmd.autokick_straight:
                    uart_command.kicker_mode = 3
                elif cmd.autokick_high:
                    uart_command.kicker_mode = 4
                else:
                    uart_command.kicker_mode = 0
            case control_pb2.RobotControlType.COORDINATE_CONTROL:
                pass
            case control_pb2.RobotControlType.GLOBAL_COORDINATES:
                tgt = aux.Point(last_cmd.global_coordinates.x,last_cmd.global_coordinates.y)
                if aux.dist(tgt,robot.pos)>0.05:
                    go_to_point_ignore(tgt,last_cmd.global_coordinates.angle,uart_command)
                else:
                    command_reciever.last_cmd = control_pb2.RobotCommand()
            case control_pb2.RobotControlType.CAP_VEL_AND_ACCEL:
                #TODO:implement
                pass
        if uart_command.kicker_mode == 1:
            # uart_command.vel_y = ball_detector.p_pos.x/640
            pnt1 = aux.closest_point_on_line(field.ball.pos,field.ball.pos+aux.RIGHT,robot.pos,"L")
            
            if aux.dist(pnt1,robot.pos)>0.2:
                go_to_point_ignore(pnt1,aux.angle_to_point(pnt1,field.ball.pos),uart_command)
            if ball_detector.p_pos is not None:
                uart_command.vel.y = ball_detector.p_pos.x/640*2
            uart_command.w = aux.wind_down_angle(aux.angle_to_point(pnt1,field.ball.pos)-robot.ang)
            uart_command.angle_mode = True
        # print(aux.get_angle_between_points(aux.rotate(aux.RIGHT,1)))
        # print("com_p:",ball_detector.r_pos)
        ser.write(uart_command.to_bytes())