import multiprocessing

import config
from ball_detector import BallDetector
from command_receiver import command_receiver
from telemetry_transceiver import telemetry_transceiver
from vision.vision_receiver import VisionClient


def main():
    client = VisionClient()
    ball_detector = BallDetector()
    # From the hostname; telemetry_transceiver changes it when the robot's buttons do
    robot_id = multiprocessing.Value("i", config.ROBOT_ID)
    processes = [
        multiprocessing.Process(target=client.run, name="Vision_Receiver"),
        multiprocessing.Process(target=command_receiver, args=(client, ball_detector, robot_id), name="Command_Sender"),
        multiprocessing.Process(
            target=telemetry_transceiver, args=(ball_detector, robot_id), name="Telemetry_Transceiver"
        ),
    ]

    for p in processes:
        p.start()

    try:
        for p in processes:
            p.join()
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()


if __name__ == "__main__":
    main()
