import socket
import serial
import config
def telemetry_tranciever():    
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        if ser.in_waiting > 0:
            packet = ser.readline() 
            if packet:
                sock.sendto(packet, ("<broadcast>", config.TEL_PORT))
