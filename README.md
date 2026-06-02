ROS 2 Kilted stack for a small Raspberry Pi 4 robot car.

To install on a Robot: 
Clone repository in pi home directory. You will want to change the user "ex123" in robot.service to your user (by default pi). Then
```console
mv ~/pi-car-ros ~/ros2_ws
sudo cp ~/ros2_ws/robot_startup.service /etc/systemd/system/ # copy startup service
sudo systemctl daemon-reload
sudo systemctl enable robot_startup.service # enable startup at boot
sudo systemctl start robot_startup.service # start immediatly
systemctl status robot_startup.service # check if successful
```

Licensed under Apache-2.0