import os

ROBOT_ID = int(os.environ.get("FB4_ROBOT_ID", 5))
ROBOT_TEAM = os.environ.get("FB4_ROBOT_TEAM", "yellow")

BAUD_RATE = 115200
UART_PORT = os.environ.get("FB4_UART_PORT", "/dev/ttyACM0")
CMD_RATE_HZ = 100  # matches the 10 ms firmware main loop
CMD_TIMEOUT_S = 0.5  # stop the robot when the strategy goes silent for this long
# Target commands (GLOBAL_COORDINATES, RICOCHET) run without the timeout above, but drive by the
# SSL vision position: stop when vision hasn't seen the robot for this long
VISION_TIMEOUT_S = 0.3

CMD_PORT = 5555
TEL_PORT = 5556
BIND_HOST = "0.0.0.0"

SHM_NAME = "ball_shm"

# RICOCHET: lateral speed per metre of ball offset seen by the onboard camera
# (ball_detector.r_pos.y, robot frame). Flip the sign if the robot runs away from the ball.
RICOCHET_CAM_GAIN = float(os.environ.get("FB4_RICOCHET_CAM_GAIN", 5.0))  # [1/s]
RICOCHET_MAX_VEL = 2.0  # [m/s]
