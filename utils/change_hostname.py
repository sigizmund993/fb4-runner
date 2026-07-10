import os
import subprocess
import sys


def change_hostname(new_hostname:str) -> None:
    subprocess.run(["hostnamectl", "set-hostname", new_hostname], check=True)
    with open("/etc/hosts", "r") as f:
        lines = f.readlines()
    with open("/etc/hosts", "w") as f:
        for line in lines:
            if "127.0.1.1" in line:
                f.write(f"127.0.1.1\t{new_hostname}\n")
            else:
                f.write(line)
    subprocess.run(
        ["systemctl", "restart", "avahi-daemon"],
        check=True,
        stdout=subprocess.DEVNULL,
    )