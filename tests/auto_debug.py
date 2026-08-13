#!/usr/bin/env python3

import getpass
import shlex
import shutil
import subprocess
from inspect import currentframe, getframeinfo
from os import getenv
from pathlib import Path
from socket import gethostname

ROVER_USER = "astra"

ROVER_IPS = [
    ("Clucky", "192.168.0.69"),  # local
    ("Clucky", "10.86.59.252"),  # eduroam
    ("Testbed", "192.168.0.70"),  # local
    ("Testbed", "10.86.125.130"),  # eduroam
]

# BatchMode makes a missing SSH key an immediate error instead of an
# invisible password prompt that hangs until the timeout; accept-new trusts
# hosts on first contact but still rejects changed keys (e.g. after a reflash)
SSH_OPTS = [
    "-o",
    "BatchMode=yes",
    "-o",
    "ConnectTimeout=5",
    "-o",
    "StrictHostKeyChecking=accept-new",
]


class Tester:
    def __init__(self):
        """Find the rover's IP, if not already running on it."""
        # os.getlogin() raises OSError without a controlling tty (e.g. when
        # this script itself is run over SSH)
        user = getpass.getuser()
        host = gethostname()

        # Force to be bool
        self.is_rover = True if getenv("ISROVER_OVERRIDE") else False

        if user == ROVER_USER and (host == "clucky" or host == "testbed"):
            # We are on the rover
            self.is_rover = True

        print(
            "This script will attempt to perform some common health checks to assist with troubleshooting."
        )
        print(
            "If not ran on the rover, it will attempt to connect over SSH and run the commands remotely."
        )

        if self.is_rover:
            print("Running on the rover.")
        else:
            # Test IPs to find the rover
            candidates = ROVER_IPS
            if override_ip := getenv("ROVER_IP_OVERRIDE"):
                candidates = [("override", override_ip)]

            # Figure out where that bitch is
            self.rover_ip = rover_name = None
            for name, ip in candidates:
                if subprocess.run(["ping", "-w", "1", ip]).returncode == 0:
                    # Can ping rover
                    self.rover_ip = ip
                    rover_name = name
                    break

            if not self.rover_ip:
                if override_ip:
                    error_result(
                        f"Unable to reach the rover at ROVER_IP_OVERRIDE={override_ip}."
                    )
                error_result("Unable to reach the rover; failed to ping all known IPs.")

            print(f"Found {rover_name} at {self.rover_ip}.")

            # Ping succeeding does not mean SSH will; fail fast with a fix
            # instead of a confusing error in the middle of the checks
            if self.run_on_rover(["true"], timeout=10)[0] != 0:
                error_result(
                    f"Cannot SSH to {ROVER_USER}@{self.rover_ip}. If you have"
                    " not set up SSH keys, run"
                    f" `ssh-copy-id {ROVER_USER}@{self.rover_ip}` first."
                )

    def run_checks(self):
        # TEST: Is Anchor service running
        if (
            self.run_on_rover(["systemctl", "--quiet", "is-active", "anchor.service"])[
                0
            ]
            != 0
        ):
            print(
                "WARN: Anchor service is not running. Continuing anyways;"
                " this is fine if you launched Anchor yourself with 'ros2 launch`."
            )

        # TEST: Is the ros2 command available
        # `command -v` needs a shell, which SSH provides; locally, ask Python
        if self.is_rover:
            ros2_found = shutil.which("ros2") is not None
        else:
            ros2_found = self.run_on_rover(["command", "-v", "ros2"])[0] == 0
        if not ros2_found:
            error_result("The ros2 command was not found on the rover.")

        # TEST: Is the anchor service running
        info_debug, info_debug_output = self.run_on_rover(
            ["ros2", "topic", "info", "/anchor/from_vic/debug"], timeout=15
        )
        if info_debug != 0:
            error_result(
                "Anchor is not running -- `ros2 topic info /anchor/from_vic/debug` failed."
            )
        else:
            # Expected format:
            # > Type: [MESSAGE_TYPE]
            # > Publisher count: [PUBS]
            # > Subscription count: [SUBS]

            lines = info_debug_output.split("\n")
            if (n := len(lines)) != 3:
                error_result(
                    f"Expected 3 lines from `ros2 topic info`, got {n}:\n"
                    f"{info_debug_output}"
                )
            if int(lines[1].split(" ")[-1]) == 0:  # No publishers
                error_result(
                    "Anchor is not running -- no publishers on /anchor/from_vic/debug."
                )

        # TEST: Are we getting any feedback from the MCU
        print("Listening for feedback from the rover...")
        # timeout runs rover-side so the echo exits cleanly; PYTHONUNBUFFERED so its
        # output isn't lost in its buffer when the timeout fires
        _, echo_output = self.run_on_rover(
            [
                "env",
                "PYTHONUNBUFFERED=1",
                "timeout",
                "5",
                "ros2",
                "topic",
                "echo",
                "/anchor/from_vic/debug",
                "--field",
                "data",
            ],
            timeout=10,
        )
        getting_feedback = False
        for line in echo_output.split("\n"):
            if line.startswith("can_relay_fromvic,"):
                getting_feedback = True
                break
        if not getting_feedback:
            error_result(
                "Not getting any feedback from the rover."
                " Try `can_relay_mode,on` (serial only)."
            )

        # END
        print(
            "All checks passed. This does not guarantee that you will not run into any problems."
        )

    def run_on_rover(self, command: list[str], timeout=60) -> tuple[int, str]:
        """Run a process on the rover, either via SSH or directly. Returns its returncode and stdout."""
        if not self.is_rover:
            command = [
                "ssh",
                *SSH_OPTS,
                f"{ROVER_USER}@{self.rover_ip}",
                f"bash -lc {shlex.quote(shlex.join(command))}",
            ]

        proc = None
        try:
            proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        except FileNotFoundError as e:
            error_result(f"Command not found: {e.filename}")

        assert proc is not None  # There is genuinely no way that proc could be None

        outs = None
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
