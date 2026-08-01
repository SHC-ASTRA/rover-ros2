#!/usr/bin/env python3

import os
import subprocess
from inspect import currentframe, getframeinfo
from socket import gethostname
from pathlib import Path

ROVER_IPS = [
    "192.168.0.69",  # Clucky (local)
    "10.86.59.252",  # Clucky (eduroam)
    "192.168.0.70",  # Testbed (local)
    "10.86.125.130",  # Testbed (eduroam)
]

# TODO: import pytest


class Tester:
    def __init__(self):
        """Find the rover's IP, if not already running on it."""
        user = os.getlogin()
        host = gethostname()

        # Force to be bool
        self.is_rover = True if os.getenv("ISROVER_OVERRIDE") else False

        if user == "astra" and (host == "clucky" or host == "testbed"):
            # We are on the rover
            self.is_rover = True

        print(
            "This script will attempt to perform some common health checks to assist with troubleshooting."
        )
        print(
            "If not ran on the rover, it will attempt to connect over SSH and run the commands remotely."
        )

        # Test IPs to find the rover
        if not self.is_rover:
            # Figure out where that bitch is
            self.rover_ip = None
            for ip in ROVER_IPS:
                ping = subprocess.run(["ping", "-w", "1", ip])

                if ping.returncode != 0:
                    continue

                # Can ping rover
                self.rover_ip = ip

            if not self.rover_ip:
                error_result("Unable to establish a connection with the rover.")

        # We are now able to connect to the rover
        if self.is_rover:
            print("Running on the rover.")
        else:
            print("Found the rover.")

    def run_checks(self):
        # TEST: Is Anchor service running
        if (
            self.run_on_rover(["systemctl", "--quiet", "is-active", "anchor.service"])[
                0
            ]
            != 0
        ):
            print("WARN: Anchor service is not running. Continuing anyways...")

        # TEST: Is the ros2 command available
        if self.run_on_rover(["ros2", "-h"])[0] != 0:
            error_result("Cannot run ros2 command on the rover.")

        # TEST: Is the anchor service running
        info_debug, info_debug_output = self.run_on_rover(
            ["ros2", "topic", "info", "/anchor/from_vic/debug"], timeout=5
        )
        if info_debug != 0:
            error_result("Anchor is not actually running.")
        else:
            # Expected format:
            # > Type: [MESSAGE_TYPE]
            # > Publisher count: [PUBS]
            # > Subscription count: [SUBS]

            lines = info_debug_output.split("\n")
            if len(lines) != 3:
                error_result("Unexpected error.")
            if int(lines[1].split(" ")[-1]) == 0:  # No publishers
                error_result("Anchor is not actually running.")

        # TEST: Are we getting any feedback from the MCU
        print("Listening for feedback from the rover...")
        _, echo_output = self.run_on_rover(
            ["ros2", "topic", "echo", "/anchor/from_vic/debug", "--field", "data"],
            timeout=5,
        )
        getting_feedback = False
        for line in echo_output.split("\n"):
            if line.startswith("can_relay_fromvic,"):
                getting_feedback = True
                break
        if not getting_feedback:
            error_result(
                "Not getting any feedback from the rover. Try `can_relay_mode,on`."
            )

        # END
        print(
            "All checks passed. This does not guarantee that you will not run into any problems."
        )

    def run_on_rover(self, command: list[str], timeout=60) -> tuple[int, str]:
        """Run a process on the rover, either via SSH or directly. Returns its returncode and stdout."""
        if not self.is_rover:
            command = ["ssh", f"astra@{self.rover_ip}", f"{" ".join(command)}"]

        outs = None
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        try:
            outs, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            outs, _ = proc.communicate()

        return proc.returncode, outs.decode().strip()


def error_result(msg, result=1):
    cf = currentframe()
    assert cf is not None and cf.f_back is not None  # Typing

    filename = Path(getframeinfo(cf).filename).name
    line_number = cf.f_back.f_lineno
    print(f"[{filename}:{line_number}] ERROR:", msg)

    exit(result)


def main():
    tester = Tester()
    tester.run_checks()


if __name__ == "__main__":
    main()
