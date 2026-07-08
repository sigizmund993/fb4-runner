import multiprocessing
import sys
# from ball_finder import ball_finder
from telemetry_tranciever import telemetry_tranciever
from command_reciever import command_reciever
# from web_dashboard import web_dashboard

def main():
    # shared_ball_pos = multiprocessing.Array('d', [0.0, 0.0])

    # ball_finder_p = multiprocessing.Process(
    #     target=ball_finder, 
    #     args=(shared_ball_pos,),
    #     name="Ball_Finder"
    # )
    
    command_p = multiprocessing.Process(
        target=command_reciever, 
        name="Command_Sender"
    )
    
    telemetry_p = multiprocessing.Process(
        target=telemetry_tranciever, 
        name="Telemetry_Tranciever"
    )

    # dashboard_p = multiprocessing.Process(
    #     target=web_dashboard,
    #     args=(shared_ball_pos),
    #     name="Web_Dashboard"
    # )

    # ball_finder_p.start()]\
    print("goal")
    # command_p.start()
    telemetry_p.start()
    # dashboard_p.start()

    try:
        # ball_finder_p.join()
        command_p.join()
        telemetry_p.join()
        # dashboard_p.join()
    except KeyboardInterrupt:
        # ball_finder_p.terminate()
        command_p.terminate()
        telemetry_p.terminate()
        # dashboard_p.terminate()

if __name__ == '__main__':
    main()
