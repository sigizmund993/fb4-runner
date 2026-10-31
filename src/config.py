import re
import socket

from utils import envfile

# Per-robot settings come from .env in the repository root (copy .env.example); a variable set
# in the environment overrides the file


def hostname_robot_id() -> int | None:
    """NN of the fb4-NN hostname, None for any other hostname."""
    match = re.fullmatch(r"fb4-(\d+)(\.local)?", socket.gethostname())
    return int(match.group(1)) if match else None


def robot_hostname(robot_id: int) -> str:
    return f"fb4-{robot_id:02d}.local"


# The robot id lives only in the hostname; the STM32 takes it over UART and doesn't keep it.
# An id changed with the robot's buttons comes back in the telemetry and renames the RPi (see telemetry_transceiver)
# Any other hostname: id 0, and telemetry_transceiver renames the RPi to fb4-00
ROBOT_ID = hostname_robot_id() or 0
MAX_ROBOT_ID = 15  # the firmware's id buttons wrap at 16
# Take the STM32's id once it has differed from ours this long: on connect the STM32 takes our
# id within a frame, so a lasting difference means it was changed with the buttons
ROBOT_ID_ADOPT_S = 1.0
ROBOT_TEAM = envfile.get("FB4_ROBOT_TEAM", "yellow")

BAUD_RATE = 115200
UART_PORT = envfile.get("FB4_UART_PORT", "/dev/ttyACM0")
CMD_RATE_HZ = 100  # matches the 10 ms firmware main loop
CMD_TIMEOUT_S = 0.5  # stop the robot when the strategy goes silent for this long
# Target commands (GLOBAL_COORDINATES, RICOCHET, CATCH_BALL) run without the timeout above, but drive by the
# SSL vision position: stop when vision hasn't seen the robot for this long
VISION_TIMEOUT_S = 0.3

KICKER_MAX_V = 330  # the firmware's capacitor charge limit
KICKER_MIN_SETTING_V = 100  # OldFormat kicker_setting 1; 15 is KICKER_MAX_V

CMD_PORT = 5555
TEL_PORT = 5556
BIND_HOST = "0.0.0.0"

SHM_NAME = "ball_shm"

# RICOCHET and CATCH_BALL: lateral speed per metre of ball offset seen by the onboard camera
# (ball_detector.r_pos.y, robot frame: y left). Positive: the robot moves towards the ball.
RICOCHET_CAM_GAIN = float(envfile.get("FB4_RICOCHET_CAM_GAIN", "5.0"))  # [1/s]
RICOCHET_MAX_VEL = 2.0  # [m/s]
