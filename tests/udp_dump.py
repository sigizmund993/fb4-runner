"""Prints every strategy command that arrives over UDP.

Runs on the RPi instead of main.py, which holds the same port: systemctl stop fb4-runner.

    make test-udp
"""
import socket
import time

from google.protobuf import text_format
from google.protobuf.message import DecodeError

import config
from ssl_packet_package.protopy.spbunited.robot import control_pb2


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((config.BIND_HOST, config.CMD_PORT))
    except OSError as e:
        raise SystemExit(f"can't listen on :{config.CMD_PORT} ({e}); stop main.py: systemctl stop fb4-runner")
    print(f"listening on :{config.CMD_PORT}, Ctrl+C to stop")

    start = time.monotonic()
    while True:
        data, (src, _) = sock.recvfrom(1024)
        stamp = f"{time.monotonic() - start:8.3f} {src}"
        cmd = control_pb2.RobotCommand()
        try:
            cmd.ParseFromString(data)
        except DecodeError:
            print(f"{stamp} not a RobotCommand ({len(data)} B): {data.hex()}")
            continue
        print(f"{stamp} {text_format.MessageToString(cmd, as_one_line=True)}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
