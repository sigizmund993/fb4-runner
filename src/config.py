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

# The defaults of the coefficients below the strategy can change on the fly (SET_COEFFICIENTS, see tuning.py)
# The STM32 coefficients are sent to it this often: it doesn't keep them over a reset
SETTINGS_RESEND_S = 1.0

# GLOBAL_COORDINATES and RICOCHET drive to the target at speed = POSITION_KP * distance, up to POSITION_MAX_VEL
POSITION_KP = 1.0  # [1/s]
POSITION_MAX_VEL = 3.2  # [m/s], the firmware's limit
# RICOCHET and CATCH_BALL: once the onboard camera sees the ball coming to the kicker, the robot moves only
# across the ball path, speed = gain * miss (from the path to the kicker centre); the gain is at least
# RICOCHET_CAM_GAIN and grows so the miss is closed by RICOCHET_FREEZE_S before the ball comes. CATCH_BALL
# before that: sideways speed RICOCHET_CAM_GAIN * y of the ball seen by the camera (robot frame: y left)
RICOCHET_CAM_GAIN = float(envfile.get("FB4_RICOCHET_CAM_GAIN", "5.0"))  # [1/s]
RICOCHET_MAX_VEL = 2.0  # [m/s]
# Ball centre at the hit, robot frame (camera calibration origin: robot centre), x forward; as the strategy's
# contact point at ROBOT_R = 100 mm from the robot centre
KICKER_FACE_X = float(envfile.get("FB4_KICKER_FACE_X", "0.10"))  # [m]
# The ball "comes to the kicker" if its speed towards the kicker centre is above this
RICOCHET_MIN_APPROACH_VEL = 0.3  # [m/s]
# The robot stops correcting this long before the ball comes
RICOCHET_FREEZE_S = 0.05  # [s]
# The ball path is a line through the camera frames of the last BALL_PATH_S, at least BALL_PATH_MIN_FRAMES
# of them (the camera runs at 30 fps)
BALL_PATH_S = 0.2  # [s]
BALL_PATH_MIN_FRAMES = 3
