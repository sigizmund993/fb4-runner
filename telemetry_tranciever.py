import socket
import serial
import config
import json
from utils.change_hostname import change_hostname
from ssl_packet_package.protopy.spbunited.robot import telemetry_pb2
def telemetry_tranciever():    
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    robot_id = 0
    change_hostname(f"fb4-{robot_id:02d}.local")
    while True:
        if ser.in_waiting > 0:
            packet = ser.readline()
            print(packet)
            json_packet = json.loads(packet)
            if json_packet["id"] != robot_id:
                robot_id = json_packet["id"]
                change_hostname(f"fb4-{robot_id:02d}.local")
            if packet:
                proto_package = telemetry_pb2.RobotTelemetry()
                proto_package.ball_deep = json_packet["ball"]["deep"]
                proto_package.ball_in = json_packet["ball"]["front"]
                proto_package.kicker_voltage = json_packet["voltage"]
                sock.sendto(proto_package.SerializeToString(), ("<broadcast>", config.TEL_PORT))