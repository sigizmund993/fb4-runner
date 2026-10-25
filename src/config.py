import os

ROBOT_ID = int(os.environ.get("FB4_ROBOT_ID", 5))
ROBOT_TEAM = os.environ.get("FB4_ROBOT_TEAM", "yellow")

BAUD_RATE = 115200
UART_PORT = os.environ.get("FB4_UART_PORT", "/dev/ttyACM0")
CMD_RATE_HZ = 100  # matches the 10 ms firmware main loop

CMD_PORT = 5555
TEL_PORT = 5556
BIND_HOST = "0.0.0.0"

SHM_NAME = "ball_shm"
