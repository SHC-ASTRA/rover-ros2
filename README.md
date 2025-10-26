# ASTRA Rover ROS2 Nodes

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

## About

Includes all ROS2 packages for the rover. These are centrally located for modular rover operation.

You will use this package to launch all module-side ROS2 nodes.

# Table of Contents

1. Title
3. Table of Contents
3. Software Requirements
    1. ROS2 Humble
    2. IP Configuration
4. Hardware Requirements
5. Recommended Programs
6. How to Use
7. Common Problems/Troubleshooting
8. Major To-Do Items
9. Author(s) 
10. Maintainer(s)

# Software Requirements 

Since July/August of 2025, ASTRA uses NixOS. A functioning NixOS installation is assumed.

## ROS2 Humble

We use [ROS2 Humble](https://docs.ros.org/en/humble/index.html), and plan on using that until support for it expires. 

To install ROS2, run the following command in your /etc/nixos folder:
```nix
nix flake init --template github:lopsided98/nix-ros-overlay
```

After you install ROS2, you can install all dependencies needed by this repository by running the following commands:

```bash
$ sudo rosdep init  # only run once
$ rosdep update
$ rosdep install --from-path . --ignore-src -y
```

## IP Configuration

- IP Address: 192.168.1.x
    - This can be just about anything not already in use. I recommend something 30-39
- Net Mask: 255.255.255.0
- Gateway: 192.168.1.0

# Hardware Requirements 

- A functioning computer.

# Recommended Programs

## VSCode

VSCode is a wonderful program. I used it to make this project and recommend anyone else working on it use it as well. 

To install VSCode, add the following package to your pkgs array / packages.nix config file:
```nix
vscode-fhs
```
Alternatively, use a flake to install home-manager, and add to your home.nix:
```nix
vscode.enable = true;
```

# How to Use

```bash
$ colcon build
$ source install/setup.bash
  # main launch files:
$ ros2 launch anchor_pkg rover.launch.py
$ ros2 run headless_pkg headless_full
```

## Testing Serial

You can fake the presence of a Serial device (i.e., MCU) by using the following command:

```bash
socat -dd -v pty,rawer,crnl,link=/tmp/ttyACM9 pty,rawer,crnl,link=/tmp/ttyOUT
```

## Connecting the GuliKit Controller

Connecting the GuliKit Controller (Recommended)

* Connect controller to pc with USB-C
* Select the "X-Input" control mode (Windows logo) on the controller.
* Hold the button next to the symbols (windows, android, switch, etc...)
* You'll need to release the button and press down again to cycle to the next mode

# Common Problems/Troubleshooting
 
- TBD

# Major To-Do Items

- TBD

# Author(s)

|Name| Email |
|--|--|
| Tristan McGinnis | tlm |
| David Sharpe | ds0196@uah.edu |

# Maintainer(s)

|Name| Email |
|--|--|
| David Sharpe | ds0196@uah.edu |
| Riley McClain | email |
