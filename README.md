# ASTRA Rover ROS2 Packages

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

Includes all main ROS2 packages for the rover. These are centrally located for modular rover operation.

You will use these packages to launch all rover-side ROS2 nodes.

## Table of Contents

- [Software Prerequisites](#software-prerequisites)
  - [Nix](#nix)
  - [ROS2 Humble + rosdep](#ros2-humble--rosdep)
  - [Docker](#docker)
- [Running](#running)
  - [Testing Serial](#testing-serial)
  - [Connecting the GuliKit Controller](#connecting-the-gulikit-controller)
- [Common Problems/Toubleshooting](#common-problemstroubleshooting)
- [Packages](#packages)
- [Maintainers](#maintainers)

## Software Prerequisites

You need either [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
with [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-installation)
or [Nix](https://nixos.org/download/#nix-install-linux) installed. We recommend
using Nix.

### Nix

With Nix, all you have to do is enter the development shell:

```bash
$ cd path/to/rover-ros2
$ nix develop
```

### ROS2 Humble + rosdep

With ROS2 Humble, start by using rosdep to install dependencies:

```bash
  # Setup rosdep
$ sudo rosdep init  # only run if you haven't already
$ rosdep update
  # Install dependencies
$ cd path/to/rover-ros2
$ rosdep install --from-paths src -y --ignore-src
```

### Docker

Using the docker compose file automatically builds the workspace and allows you to choose between running on the CPU or GPU for applications like Rviz2 and Gazebo:

```bash
  # Run on CPU
$ docker compose run --rm --name rover-ros2-container cpu
  # Run on GPU (NVidia only)
$ docker compose run --rm --name rover-ros2-container gpu
```

## Running

```bash
$ colcon build
$ source install/setup.bash
  # main launch files:
$ ros2 launch anchor_pkg rover.launch.py  # Must be run on a computer connected to a MCU on the rover.
$ ros2 run headless_pkg headless_full  # Optionally run in a separate shell on the same or different computer.
```

### Testing Serial

You can fake the presence of a Serial device (i.e., MCU) by using the following command:

```bash
$ socat -dd -v pty,rawer,crnl,link=/tmp/ttyACM9 pty,rawer,crnl,link=/tmp/ttyOUT
```

When you go to run anchor, use the `PORT_OVERRIDE` environment variable to point it to the fake serial port, like so:

```bash
$ PORT_OVERRIDE=/tmp/ttyACM9 ros2 launch anchor_pkg rover.launch.py
```

### Connecting the GuliKit Controller

These instructions apply to the black XBox-style GuliKit controller, primarily used for controlling Arm through Basestation.

* Connect the controller to your PC with a USB-C cable
* Select the "X-Input" control mode (Windows logo) on the controller.
  * Hold the button next to the symbols (windows, android, switch, etc...)
  * You'll need to release the button and press down again to cycle to the next mode

## Common Problems/Troubleshooting

**Q**: When I try to launch the nodes, I receive a `package '' not found` error.

A: Make sure you have sourced the workspace in the current shell:

```bash
$ source install/setup.bash  # or setup.zsh if using ZSH
```

**Q**: When I try to launch the nodes, I receive several `FileNotFoundError: [Errno 2]` errors.

A: Sometimes the install files get messed up by running `colcon build` in different shells or updating packages. Try running the following commands to clean up your local build files:

```bash
$ rm -rf build/ install/
$ colcon build
```

**Q**: When I run `colcon build` after the above suggestion, I receive several of the following errors:

```bash
[0.557s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '' in the environment variable AMENT_PREFIX_PATH doesn't exist
```

A: Don't worry about it. If you had the workspace sourced, ROS2 will complain about the workspace install files not existing anymore after you deleted them. They will be re-created by `colcon build`, after which you can run `source install/setup.bash` to source the new install files.

**Q**: When I try to launch Anchor, I receive the following errors:

```bash
[anchor-5] [INFO] [1762239452.937881841] [anchor]: Unable to find MCU...
...
[ERROR] [anchor-5]: process has died [pid 101820, exit code 1, cmd '.../rover-ros2/install/anchor_pkg/lib/anchor_pkg/anchor --ros-args -r __node:=anchor --params-file /tmp/launch_params_nmv6tpw4'].
[INFO] [launch]: process[anchor-5] was required: shutting down launched system
[INFO] [bio-4]: sending signal 'SIGINT' to process[bio-4]
[INFO] [ptz-3]: sending signal 'SIGINT' to process[ptz-3]
[INFO] [core-2]: sending signal 'SIGINT' to process[core-2]
[INFO] [arm-1]: sending signal 'SIGINT' to process[arm-1]
...
```

A: To find a microcontroller to talk to, Anchor sends a ping to every Serial port on your computer. If it does not receive a 'pong' in less than one second, then it aborts. There are a few possible fixes:

- Keep trying to run it until it works
- Run `lsusb` to see if the microcontroller is detected by your computer.
- Run `ls /dev/tty*0` to see if there is a valid Serial port enumerated for the microcontroller.
- Check if you are in the `dialout` group (or whatever group shows up by running `ls -l /dev/tty*`).

## Packages

- [anchor\_pkg](./src/anchor_pkg) - Handles Serial communication between the various other packages here and the microcontroller.
- [arm\_pkg](./src/arm_pkg) - Relays controls and sensor data for the arm (socket and digit) between anchor and basestation/headless.
- [astra\_descriptions](./src/astra_descriptions) - Submodule with URDF-related packages.
- [bio\_pkg](./src/bio_pkg) - Like arm_pkg, but for CITADEL and FAERIE
- [core\_pkg](./src/core_pkg) - Like arm_pkg, but for Core
- [headless\_pkg](./src/headless_pkg) - Simple, non-graphical controller node to work in place of basestation when controlling the rover by itself. This is autostarted with anchor to allow for setup-less control of the rover.
- [latency\_tester](./src/latency_tester) - A temporary node to test comms latency over ROS2, Serial, and CAN.
- [ros2\_interfaces\_pkg](./src/ros2_interfaces_pkg) - Contains custom message types for communication between basestation and the rover over ROS2. (being renamed to `astra_msgs`).
- [servo\_arm\_twist\_pkg](./src/servo_arm_twist_pkg) - A temporary node to translate controller state from `ros2_joy` to `Twist` messages to control the Arm via IK.

## Maintainers

| Name | Email | Discord |
| ---- | ----- | ------- |
| David Sharpe | <ds0196@uah.edu> | `@ddavdd` |
| Riley McLain | <rjm0037@uah.edu> | `@ryleu` |
