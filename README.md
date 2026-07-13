# PiCar-ROS

ROS 2 Kilted control stack for a Raspberry Pi 4 robot car.

This repository provides a fully implemented control stack using ROS 2 Kilted through
multiple ROS modules. At the heart of this software is the `robot_hat_driver` module which directly
interfaces with the robot hardware. On top of this sit a detailed robot description, automated bringup and
controller modules implementing line following and object avoidance.
Although this project is targeted at the SunFounder PiCar-X it could be adapted to other small robots
using the SunFounder Robot HAT.

## Installation

To install this control stack on a robot, a Ubuntu 24.04 OS and a [ROS 2 Kilted installation](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
on the Raspberry Pi are required. Alternatively, using containerisation through tools like distrobox
is possible but might degrade performance on low-spec systems.
Clone the repository into the Raspberry Pis home directory and rename via

```console
mv ~/pi-car-ros ~/ros2_ws
```

Install ROS dependencies and build the repository:

```console
source /opt/ros/kilted/setup.bash && cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

Then source it and start the control stack with:

```console
source install/setup.bash
ros2 launch picar_startup bringup.launch
```

If you want to automatically restart the control stack on boot (recommended), you can use the robot_startup.service.
You will want to change the user "ex123" in robot_startup.service to your user (by default pi). Then:

```console
sudo cp ~/ros2_ws/robot_startup.service /etc/systemd/system/ # copy startup service
sudo systemctl daemon-reload
sudo systemctl enable robot_startup.service # enable startup at boot
sudo systemctl start robot_startup.service # start immediatly, this takes a while
systemctl status robot_startup.service # check if successful
```

## Running on PiCar

The software is configured to work out of the box with the standard Sunfounder PiCar setup but is
configurable through a number of config files in `src/"package"/config/"name".yaml`. It does not
make use of the moving camera attachment but relies on the motors, the steering servo as well as the
sonar and grayscale sensors to function optimally.

By default, launching picar_startup/bringup.launch will start a line following setup that can be
toggled via the USR button. To calibrate the the grayscale IR sensor, place the sensor above a white
surface and press the RST button. The indicator LED on the HAT will flash to show that calibration
has begun, and after half a second turn on for 5 seconds. In this window, move the robot onto a dark
surface. The LED will flash again after calibration is complete. The default calibration may not
necessarily work for your sensor.

## Running on Steamdeck

The project provides additional support for robot teleoperation through a connected Steamdeck. To
make use of this feature, ROS2 Kilted needs to be installed the controlling Steamdeck.
To install ROS2 Kilted on the Steamdeck without reinstalling a different OS it is recommended to
use the distrobox tool to create a Ubuntu 24.04 container. The container name used in scripts in
this project is `ros2-kilted` and is thus recommended.
Within the distrobox, install Kilted and follow the repository setup like usual. After building the
project, execute the file `~/ros2_ws/src/picar_steamdeck/install.sh` to add the launch and kill
scripts as icons on the desktop.

To start the controller, connect to the target robot via wifi and launch the controller node via the
icon on the desktop. To shut down the controller, use the Terminate icon on the desktop. (If no input
is registered, make sure that Steam is not running and capturing input in the background)

Controls:

- Left stick: Drive forward/backward
- Right stick: Steer left/right
- L1: Dead man's switch (slow, 40% speed)
- R1: Dead man's switch (fast, 100% speed)
- X/Y: Enable/Disable sensors
- A/B: Line following/Teleoperation driving modes
- (…): Start grayscale sensor calibration sequence

## Structure

This repository is a complete ROS2 Kilted workspace with supplementary files. The `src` folder
contains six packages that together allow the operation of the Sunfounder PiCar as a fully
ROS2 integrated robot.

### picar_description

A minimal package containing a `.urdf` description of the PiCar robot using the CMake build system.
This description provides a tree of transforms and joints that represent an accurate model of the
PiCar. Also helpful for visualization. This project is set up to automatically expand xacro macros
in the description file (hence the additional `.xacro` file ending) but none are currently used.

### picar_interfaces

A library package defining interfaces used in other packages:

- `ServoCmd` message, message type specifiying a target servo channel and angle
- `UtilitySrv` service, service type for handling command strings, reports success as a bool and an optional message string

### picar_startup

This package contains the launch file used to start and correctly configure the entire ROS setup as
one, using the ament_python build system.

### picar_controller

This package contains an assortment of nodes to control the higher functions of the robot, like
autonomous driving, teleoperation and state reporting.
In detail, there are the `state_publisher` node which updates the robot transform tree, the
`utility_service` node for handling commands sent from a controlling device (Steamdeck), the
`line_follower` node that automatically drives along a line using the grayscale sensor as well as
the `teleop` node that translates teleoperation control input into hardware commands.

Currently the only job of the `state_publisher` is to update the steering servo angles on the
transform tree.

- Publishes: `JointState /joint_states`
- Broadcasts: `TransformBroadcaster`
- Subscribes: `Float64MultiArray /servo_angles`

The `utility_service` node advertises a service that translates command strings from a controlling
device into hardware commands. This architecture helps reduces complexity and increase reusability
and resilience in the Steamdeck controller logic.

- Services: `/picar_interfaces/srv/UtilitySrv /picar_utility`
- Clients:
  - `std_srvs/srv/SetBool /follow_line`
  - `std_srvs/srv/SetBool /teleop_control`
  - `std_srvs/srv/SetBool /set_sensors`
  - `std_srvs/srv/Trigger /calibrate_grayscale`

The `line_follower` node is more involved. It starts by default but is in a disabled state that must
be enabled via a service call or, if configured so, through an `Empty` message on the `/usr_button`
topic. The node may additionally be e-stopped if no grayscale sensor input has been received for
a specified length of time (default: 0.5s). If disabled or e-stopped, the node does nothing. If
enabled, the node will process sensor data from the grayscale sensor, attempt line recognition
and send new motor speed and steering commands. If no line is found (line not present, insufficient
contrast, broken calibration…) the node will signal this by turning on the indicator LED on the HAT.

- Publishes:
  - `std_msgs/msg/Float64 /motor_speed`
  - `/picar_interfaces/msg/ServoCmd /servo_targets`
  - `std_msgs/msg/Bool /robot_hat_led`
- Subscribes:
  - `sensor_msgs/msg/Image /grayscale` (3x1 mono16)
  - `std_msgs/msg/Empty /usr_button` (if parameter `button_toggle: True`)
  - `std_msgs/msg/Empty /rst_button` (if parameter `button_toggle: True`)
- Services:
  - `std_srvs/srv/SetBool /follow_line` (enable/disable line following)
- Clients:
  - `std_srvs/srv/Trigger /calibrate_grayscale` (triggers grayscale calibration sequence)

Finally, the `teleop` node takes in `Twist` command velocity messages and translates these into motor
speed and servo steering messages. This node, like `line_follower`, is disabled by default and needs
to be enabled first, likely through triggering by the controller. It also features a watchdog that,
when teleoperation is enabled, stops the vehicle after a timeout from the controller, preventing
losing control of the robot upon controller disconnects. This timeout can be changed, but a short time is
recommended.

All nodes in the picar_controller package take in parameters from the same config file. Here with
defaults:

```yaml
state_publisher:
  ros__parameters:
line_follower:
  ros__parameters:
    timeout_sec: 0.5 # seconds without grayscale input until e-stop
    max_steer_angle: 15.0 # maximum servo steering angle
    line_inverted: False # Is the line white on black?
    direction_history_length: 5 # length of history to keep for directions, noise reduction
    button_toggle: False # Subscribe to button topics for enabling and calibration?
utility_service_node:
  ros__parameters:
teleop_node:
  ros__parameters:
    timeout_sec: 0.1 # controller timeout
    max_steer_angle: 20.0 # max range (-45) to 45
    max_speed: 100.0 # max range 0 to 100
```

### robot_hat_driver

This package interfaces with the Robot HAT hardware and exposes relevant topics. Some code has been
adapted from [sunfounder/robot-hat](https://github.com/sunfounder/robot-hat). It contains the
`rhdriver` node that manages all hardware interaction.

- Publishes:
  - `sensor_msgs/msg/BatteryState /battery_state`
  - `sensor_msgs/msg/Range /sonar_range` (if sonar sensor configured)
  - `sensor_msgs/msg/Image /grayscale`   (if grayscale sensor configured)
  - `std_msgs/msg/Float64MultiArray /servo_angles`
  - `std_msgs/msg/Empty /usr_button` (publishes Empty when button released)
  - `std_msgs/msg/Empty /rst_button` (publishes Empty when button released)
- Subscribes:
  - `std_msgs/msg/Bool /robot_hat_led` (control indicator LED)
  - `/picar_interfaces/msg/ServoCmd /servo_targets` (control servos)
  - `std_msgs/msg/Float64 /motor_speed` (control motor_speed)
- Services:
  - `std_srvs/srv/Trigger /calibrate_grayscale` (if grayscale sensor configured)
  - `std_srvs/srv/SetBool /set_sensors` (enable/disable sensors)

Parameters loaded from config file with default values:

```yaml
robot_hat_node:
  ros__parameters:
    max_motor_percent: 100.0 # maximum turning speed of motors, 0-100
    motor_left_id: 1
    motor_right_id: 2 # connected motor port numbers
    motor_left_reversed: False
    motor_right_reversed: False
    servo_channels: ["-1"] # servo PWM channels as strings
    servo_correction: [0.0] # Correction angles for servos
    ultrasonic_sensor: False # sonar sensor present?
    ultrasonic_pins: ["-1"] # sonar sensor GPIO pins
    grayscale_sensor: False # grayscale sensor present?
    grayscale_pins: ["-1"] # grayscale sensor GPIO pins
    grayscale_calibration: [1495, 1481, 1457, 1429, 1378, 1067]
```

### picar_steamdeck

This package provides a way to use the Steamdeck as a controller for ROS2 robots. It uses the `joy`
and `teleop_twist_joy` packages to translate joystick input into command velocities and it provides
the `deckin` node to turn button presses into utility commands. Additionally, it includes scripts
to work with ROS2 on unsupported OSs (i.E. Steam OS) through distrobox. See "Running on Steamdeck"
for information on the installation process.

Parameters loaded from config file with default values:

```yaml
deck_input_node:
  ros__parameters:
    timeout_sec: 5.0
    cmds: 
      - "99:undefined_cmd" # Commands are given as button_id:command_string with spaces
```

## Roadmap

This project is considered to be completed:

- [x] Robot HAT driver module
  - [x] Hardware checks
  - [x] Servo control
  - [x] Motor control
  - [x] Battery reporting
  - [x] Sensor support
- [x] Robot description
  - [x] Accurate transform tree
  - [x] 3D model
- [x] Robot controller
  - [x] Joint state publisher
  - [x] Transform
  - [x] Line following
  - [x] Object avoidance
- [x] Teleoperation
  - [x] Steamdeck support

Licensed under Apache-2.0
