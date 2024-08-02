# rover_ros_driver
The Demo Rover is a high-performance RC car created by [CanEduDev](https://www.canedudev.com/), a side venture of Kvaser.
This repository aims to integrate CAN and ROS2 communications.

## discription
| Input | topic | value
| ---- | ---- | --- 
| throttle | /throttle_send | float32 0~100
| steering | /steering_send | float32 -45~45
| battery | /battery_send | bool 

## 1. requirements
- ubuntu (ubuntu22.04 is tested )
- Kvaser SDK, Linux SDK Library 
- CAN cable

## 2. setup
1. install Kvaser SDK
Download [here](https://kvaser.com/single-download/?download_id=47147)
```shell
cd ~/Downloads
tar xvzf linuxcan_5_45_724.tar.gz 
cd linuxcan
make
sudo make install 
```

2. install Linux SDK Library
Download [here](https://kvaser.com/single-download/?download_id=47184)
```shell
cd ~/Downloads
tar xvzf kvlibsdk_5_45_724.tar.gz 
cd kvlibsdk/
make 
sudo make install
```

I recommend system reboot.

3. build and clone this repojitory
```shell
# make ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/iASL-Gifu/rover_ros_driver.git

#build
cd ../
colcon build --symlink-install --packages-select rover_ros_driver
```

## 3. run
```shell
cd ~/ros2_ws
source install/setup.bash

ros2 launch rover_ros_driver rover_ros_driver.launch.xml
```
