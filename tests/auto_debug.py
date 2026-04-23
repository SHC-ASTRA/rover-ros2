#!/usr/bin/env python3

from inspect import currentframe, getframeinfo
import os
import socket
import subprocess
from typing import *

# TODO: import pytest


class Tester():
    isRover = False

    def __init__(self):
        user = os.getlogin()
        host = socket.gethostname()

        self.isRover = True if os.getenv("ISROVER_OVERRIDE") else False
    
        if user == "astra" and (host == "clucky" or host == "testbed"):
            # We are on the rover
            self.isRover = True

        print("This script will attempt to perform some common health checks to assist with troubleshooting.")
        print("If not ran on the rover, it will attempt to connect over SSH and run the commands remotely.")

        if not self.isRover:
            # Figure out where that bitch is
            self.rover_ip = None
            for ip in ["192.168.0.69", "192.168.0.70", "10.86.59.252", "10.86.125.130"]:
                ping = subprocess.run(["ping", "-w", "1", ip])

                if ping.returncode != 0:
                    continue

                # Can ping rover
                self.rover_ip = ip

            if not self.rover_ip:
                error_result("Unable to establish a connection with the rover.")

        if self.isRover:
            print("Running on the rover.")
        else:
            print("Connected to rover.")

        # TEST: Is Anchor service running
        if self.run_on_rover(["systemctl", "--user", "--quiet", "is-active", "anchor.service"])[0].returncode != 0:
            print("WARN: Anchor service is not running. Continuing anyways...")

        # TEST: Is the ros2 command available
        if self.run_on_rover(["ros2", "-h"])[0].returncode != 0:
            error_result("Cannot run ros2 command on the rover.")

        # TEST: Is the anchor service running
        info_debug, info_debug_output = self.run_on_rover(["ros2", "topic", "info", "/anchor/from_vic/debug"], timeout=5)
        if info_debug.returncode != 0:
            error_result("Anchor is not actually running.")
        else:
            # Expected format:
            # > Type: [MESSAGE_TYPE]
            # > Publisher count: [PUBS]
            # > Subscription count: [SUBS]

            lines = info_debug_output.strip().split("\n")
            if len(lines) != 3:
                error_result("Unexpected error.")
            if int(lines[1].split(" ")[-1]) == 0:  # No publishers
                error_result("Anchor is not actually running.")

        # TEST: Are we getting any feedback from the MCU
        print("Listening for feedback from the rover...")
        _, echo_output = self.run_on_rover(["ros2", "topic", "echo", "/anchor/from_vic/debug", "--field", "data"], timeout=5)
        getting_feedback = False
        for line in echo_output.strip().split("\n"):
            if line.startswith("can_relay_fromvic,"):
                getting_feedback = True
                break
        if not getting_feedback:
            error_result("Not getting any feedback from the rover. Try `can_relay_mode,on`.")

        # END
        print("All checks passed.")


    def run_on_rover(self, command: List[str], timeout=60):
        if not self.isRover:
            command = ["ssh", f"astra@{self.rover_ip}", f"{" ".join(command)}"]

        outs = None
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        try:
            outs, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            outs, _ = proc.communicate()

        return proc, outs.decode()


def error_result(msg, result=1):
    cf = currentframe()
    if cf is None or cf.f_back is None:  # Typing
        exit(1)
    print(f"[{getframeinfo(cf).filename.split("/")[-1]}:{cf.f_back.f_lineno}] ERROR:", msg)
    exit(result)


def main():
    Tester()


if __name__ == "__main__":
    main()
