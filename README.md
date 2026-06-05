# ROS 2 Kilted control stack for a Raspberry Pi 4 robot car.

This repository is made for the SunFounder PiCar-X but could be adapted to other small robots using
the SunFounder Robot Hat. It provides a fully implemented control stack using ROS 2 Kilted through
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

## Roadmap

- [x] Robot Hat driver module
    - [x] Hardware checks
    - [x] Servo control
    - [x] Motor control
    - [x] Battery reporting
    - [ ] Sensor support
    - [ ] Odometry estimation
- [x] Robot description
    - [x] Accurate transform tree
    - [ ] 3D model
    - [ ] Rviz2 config
- [ ] Robot controller
    - [x] Joint state publisher
    - [ ] Transform
    - [ ] Ackerman controller
    - [ ] Line following
    - [ ] Object avoidance
- [ ] Teleoperation
    - [ ] Steamdeck support

Licensed under Apache-2.0