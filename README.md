# Controlling PCA9685 with ROS2

CONTENTS OF THIS FILE
---------------------
1. README.md
2. src

Introduction
---------------------

- Tested with:
1. Raspberry Pi 4B (4 GB) with Sandisk Extreme Pro (64 GB)
2. ROS2 Foxy
3. PCA9685


Requirements
---------------------
1. ROS2
2. PCA9685

Installation
---------------------

### Pre-requisite
1. Install adafruit pca9685 and servokit library
```
sudo pip3 install adafruit-pca9685
```
```
sudo pip3 install adafruit-circuitpython-servokit
```

2. Build workspace
```
cd <ws>
colcon build
```

Usage
---------------------
```
cd <ws>
source ./install/setup.bash
ros2 run ros2_pca9685 listener
```

Additional Information
---------------------
1. This code was created to produce PWM for a rotor that generates a maximum thrust of 15.0N

2. The mapping can be reconfigured by editing the <subscriber_member_function.py> file



FAQ / Troubleshooting
---------------------
### TBC


Maintainers
---------------------
1. Tan Jun Kiat (junkiat@hotmail.com)
