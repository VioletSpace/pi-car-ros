# PiCar-ROS
ROS 2 Kilted control stack for a Raspberry Pi 4 robot car. 

This repository is made for the SunFounder PiCar-X but could be adapted to other small robots using
the SunFounder Robot HAT. It provides a fully implemented control stack using ROS 2 Kilted through
multiple ROS modules. At the heart of this software is the `robot_hat_driver` module which directly
interfaces with the robot hardware. On top of this sit a detailed robot description, automated bringup and
controller modules implementing line following and object avoidance.

## Installation
To install this control stack on a robot, a Ubuntu 24.04 OS and a ROS 2 Kilted
installation are required. Clone the repository into the Raspberry Pis home directory and rename via
```console
mv ~/pi-car-ros ~/ros2_ws
```
Install dependencies, build the repository
```console
source /opt/ros/kilted/setup.bash && cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```
Then source it and start the control stack with
```console
source install/setup.bash
ros2 launch picar_startup bringup.launch
```
If you want to automatically restart the control stack on boot (recommended), you can use the robot_startup.service.
You will want to change the user "ex123" in robot_startup.service to your user (by default pi). Then
```console
sudo cp ~/ros2_ws/robot_startup.service /etc/systemd/system/ # copy startup service
sudo systemctl daemon-reload
sudo systemctl enable robot_startup.service # enable startup at boot
sudo systemctl start robot_startup.service # start immediatly
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

## Structure

This repository is a complete ROS2 Kilted workspace with supplementary files. The `src` folder
contains four packages that together allow the operation of the Sunfounder PiCar as a fully
ROS2 integrated robot.

### picar_description

A minimal package containing a `.urdf` description of the PiCar robot using the CMake build system.
This description provides a tree of transforms and joints that represent an accurate model of the
PiCar. Also helpful for visualization. This project is set up to automatically expand xacro macros
in the description file (hence the additional `.xacro` file ending) but none are currently used.

### picar_startup

This package contains the launch file used to start and correctly configure the entire ROS setup as
one, using the ament_python build system.

### picar_controller

This package contains the `state_publisher` node that updates the robot transform tree as well as
the `line_follower` node that automatically drives along a line using the grayscale sensor.

Currently the only job of the `state_publisher` is to update the steering servo angles on the
transform tree.
- Publishes: `JointState /joint_states`
- Broadcasts: `TransformBroadcaster`
- Subscribes: `Float64MultiArray /servo_angles`

The line follower node is more involved. It starts by default but is in a disabled state that must
be enabled via a service call or, if configured so, through an `Empty` message on the `/usr_button`
topic. The node may additionally be e-stopped if no grayscale sensor input has been received for
a specified length of time (default: 0.5s). If disabled or e-stopped, the node does nothing. If
enabled, the node will process sensor data from the grayscale sensor, attempt line recognition
and send new motor speed and steering commands. If no line is found (line not present, insufficient
contrast, broken calibration…) the node will signal this by turning on the indicator LED on the HAT.
- Publishes: 
    - `std_msgs/msg/Float64 /motor_speed`
    - `std_msgs/msg/Float64MultiArray /servo_target_angles`
    - `std_msgs/msg/Bool /robot_hat_led`
- Subscribes:
    - `sensor_msgs/msg/Image /grayscale` (3x1 mono16)
    - `std_msgs/msg/Empty /usr_button` (if parameter `button_toggle: True`)
    - `std_msgs/msg/Empty /rst_button` (if parameter `button_toggle: True`)
- Services:
    - `std_srvs/srv/SetBool /follow_line` (enable/disable line following)
- Clients:
    - `std_srvs/srv/Trigger /calibrate_grayscale` (triggers grayscale calibration sequence)

Both nodes in the picar_controller package take in parameters from the config file. Here with
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
```

### robot_hat_driver

This package interfaces with the Robot HAT hardware and exposes relevant topics. Some code has been
adapted from [sunfounder/robot-hat](https://github.com/sunfounder/robot-hat). It contains the `rhdriver` node that manages all hardware interaction.
- Publishes:
    - `sensor_msgs/msg/BatteryState /battery_state`
    - `sensor_msgs/msg/Range /sonar_range` (if sonar sensor configured)
    - `sensor_msgs/msg/Image /grayscale`   (if grayscale sensor configured)
    - `std_msgs/msg/Float64MultiArray /servo_angles`
    - `std_msgs/msg/Empty /usr_button` (publishes Empty when button released)
    - `std_msgs/msg/Empty /rst_button` (publishes Empty when button released)
- Subscribes:
    - `std_msgs/msg/Bool /robot_hat_led` (control indicator LED)
    - `std_msgs/msg/Float64MultiArray /servo_target_angles` (control servos)
    - `std_msgs/msg/Float64 /motor_speed` (control motor_speed)
- Services:
    - `std_srvs/srv/Trigger /calibrate_grayscale` (if grayscale sensor configured)

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
    ultrasonic_sensor: False # sonar sensor present?
    ultrasonic_pins: ["-1"] # sonar sensor GPIO pins
    grayscale_sensor: False # grayscale sensor present?
    grayscale_pins: ["-1"] # grayscale sensor GPIO pins
    grayscale_calibration: [1495, 1481, 1457, 1429, 1378, 1067]
```

## Roadmap

- [x] Robot HAT driver module
    - [x] Hardware checks
    - [x] Servo control
    - [x] Motor control
    - [x] Battery reporting
    - [x] Sensor support
    - [ ] Odometry estimation
- [x] Robot description
    - [x] Accurate transform tree
    - [x] 3D model
    - [ ] Rviz2 config
- [x] Robot controller
    - [x] Joint state publisher
    - [x] Transform
    - [x] Line following
    - [x] Object avoidance
- [x] Teleoperation
    - [x] Steamdeck support

Licensed under Apache-2.0