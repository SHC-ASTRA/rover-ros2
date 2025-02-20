# rover-ros2

Submodule which includes all ros2 packages for the rover. These are centrally located for modular rover operation.

You will use this package to launch any module-side ROS2 nodes.
<br>
#### Table of Contents

- [Software Prerequisites](##Software-Pre-reqs)
- [Launching with ANCHOR](##Launching-with-ANCHOR)
- [Launching as Standalone](##Launching-as-Standalone)
- [Running Headless](##Running-Headless)
- [Connecting the GuliKit Controller](##Connecting-the-GuliKit-Contoller)


## Software Pre-reqs

An acting base station computer will need several things:

* ROS2 Humble
    * Follow the standard ROS2 humble install process. Linux recommended.
        * https://docs.ros.org/en/humble/Installation.html
* Colcon
    * `$ sudo apt update`
    * `$ sudo apt install python3-colcon-common-extensions`
* Configured Static IP for Ubiquiti bullet (Process varies by OS)
    * IP Address: 192.168.1.x
        * This can be just about anything not already in use. I recommend something 30-39
    * Net Mask: 255.255.255.0
    * Gateway: 192.168.1.0

## Launching with ANCHOR

ANCHOR (Active Node Controller Hub and Operational Relay)
Allows for launching all nodes on the rover simulataneously. Additionally, all controls will run through the core's NUC and MCU.
<br>
1. SSH to core
    * Core1: `ssh clucky@192.168.1.69`
    * Core2: `ssh clucky@192.168.1.70`
    * Password: \<can be found in the rover-Docs repo under documentation>
2. Navigate to rover-ros2 workspace
    * `cd rover-ros2`
3. Source the workspace
    * `source install/setup.bash`
4. Launch ANCHOR
    * `ros2 launch rover_launch.py mode:=anchor`

## Launching as Standalone

For use when running independent modules through their respective computers (pi/NUC) without ANCHOR.

1. SSH to the the module's computer
    * Core1: `ssh clucky@192.168.1.69`
    * Core2: `ssh clucky@192.168.1.70`
    * Arm: `ssh arm@192.168.1.70`
    * Bio: \<TBD>
    * Password: \<can be found in the rover-Docs repo under documentation>
2. Run the main node (this sends commands to the MCU)
    * Navigate to the rover-ros2 workspace (location may vary)
        * `cd rover-ros2`
    * Source the workspace
        * `source install/setup.bash`
    * Start the node
        * ARM: `ros2 launch rover_launch.py mode:=arm`
        * CORE: `ros2 launch rover_launch.py mode:=core`
        * BIO: `ros2 launch rover_launch.py mode:=bio`

## Running Headless

Headless control nodes (for ARM and CORE) allow running of the module on-rover without the operator having ROS2 installed on their machine. You will need a laptop to connect to the pi/NUC in order to launch headless but it can be disconnected after the nodes are spun up.
<br>
1. SSH to the the module's computer
    * Core1: `ssh clucky@192.168.1.69`
    * Core2: `ssh clucky@192.168.1.70`
    * Arm: `ssh arm@192.168.1.70`
    * Password: \<can be found in the rover-Docs repo under documentation>
2. Run the  headless node
    * You must have ANCHOR or the module's Standalone node running
    * Open a new terminal (SSH'd to the module)
    * Navigate to rover-ros2 workspace
        * `cd rover-ros2`
    * Source the workspace
        * `source install/setup.bash`
    * Run the node (ensure controller is connected and on x-input mode)
        * CORE: `ros2 run core_pkg headless`
        * ARM: `ros2 run arm_pkg headless`

## Connecting the GuliKit Controller

Connecting the GuliKit Controller (Recommended)

* Connect controller to pc with USB-C
* Select the "X-Input" control mode (Windows logo) on the controller.
* Hold the button next to the symbols (windows, android, switch, etc...)
* You'll need to release the button and press down again to cycle to the next mode