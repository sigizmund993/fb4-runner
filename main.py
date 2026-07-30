import multiprocessing
import sys
# from ball_finder import ball_finder
from telemetry_tranciever import telemetry_tranciever
from command_reciever import command_reciever
# from web_dashboard import web_dashboard
from vision_reciever import VisionClient
from ball_detector import BallDetector
def main():
    client = VisionClient()
    ball_detector = BallDetector()
    vision_p = multiprocessing.Process(
        target=client._read_loop,
        name="Vision_Reciever"
    )
    
    command_p = multiprocessing.Process(
        target=command_reciever, 
        args=(client,ball_detector,),
        name="Command_Sender"
    )
    
    telemetry_p = multiprocessing.Process(
        target=telemetry_tranciever, 
        args=(ball_detector,),
        name="Telemetry_Tranciever"
    )

    # dashboard_p = multiprocessing.Process(
    #     target=web_dashboard,
    #     name="Web_Dashboard"
    # )

    command_p.start()
    telemetry_p.start()
    vision_p.start()
    # dashboard_p.start()

    try:
        command_p.join()
        telemetry_p.join()
        vision_p.join()
        # dashboard_p.join()
    except KeyboardInterrupt:
        command_p.terminate()
        telemetry_p.terminate()
        vision_p.terminate()
        # dashboard_p.terminate()

if __name__ == '__main__':
    main()
