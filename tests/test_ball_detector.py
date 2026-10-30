"""Onboard camera ball position: pixels -> robot frame (x forward, y left)."""
import unittest

import helpers

helpers.require_protobuf()

try:
    import cv2  # noqa: F401
except ImportError:
    raise unittest.SkipTest("OpenCV unavailable")

import config  # noqa: E402
from ball_detector import CALIB_DIR, calib_to_robot_frame  # noqa: E402
from command_receiver import build_uart_command  # noqa: E402
from ssl_packet_package.protopy.spbunited.robot import control_pb2  # noqa: E402
from to_real_coords.coord_converter import CoordConverter  # noqa: E402
from utils.auxiliary import aux  # noqa: E402
from vision.vision import Robot  # noqa: E402

W, H = 1280, 960


class BallFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        converter = CoordConverter(str(CALIB_DIR / "camera_params_fisheye.npz"), str(CALIB_DIR / "homography_matrix.npy"))
        cls.to_robot = staticmethod(lambda u, v: calib_to_robot_frame(converter.get_coords(aux.Point(u, v))))

    def test_axes(self):
        center = self.to_robot(W // 2, H // 2)
        self.assertGreater(center.x, 0, "the camera looks forward")
        self.assertAlmostEqual(center.y, 0, delta=0.03)
        self.assertGreater(self.to_robot(W // 2, H // 6).x, center.x, "the top of the image is further away")
        self.assertGreater(self.to_robot(W // 6, H // 2).y, 0.05, "the left of the image is the robot's left")
        self.assertLess(self.to_robot(W * 5 // 6, H // 2).y, -0.05)

    def test_ricochet_moves_towards_the_ball(self):
        # Robot unseen by vision: ricochet only follows the ball
        cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=control_pb2.RobotControlType.RICOCHET)
        cmd.ricochet.x = cmd.ricochet.y = cmd.ricochet.angle = 0.0
        robot = Robot(aux.Point(0, 0), 0, 5, None)
        self.assertGreater(config.RICOCHET_CAM_GAIN, 0)
        for u, side in ((W // 6, 1), (W * 5 // 6, -1)):
            with self.subTest(u=u):
                uart, _ = build_uart_command(cmd, robot, self.to_robot(u, H // 2))
                self.assertEqual(uart.vel_y > 0, side > 0, "+vel_y is left")


if __name__ == "__main__":
    unittest.main()
