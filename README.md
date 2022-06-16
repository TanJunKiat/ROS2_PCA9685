# Controlling PCA9685 with ROS2

CONTENTS OF THIS FILE
---------------------
1. README.md

Introduction
---------------------
### TBC

- Tested with Raspberry Pi 4B (4 GB) with Sandisk Extreme Pro (64 GB)

Requirements
---------------------
1. Ubuntu 20.04 Desktop or Ubuntu 20.04 Mate
2. Gazebo

Installation
---------------------

### Pre-requisite
1. Install Curl:
```
sudo snap install curl
```
2. Install [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html):
```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Install the Debian package which depends on colcon-core:
```
sudo apt update
sudo apt install python3-colcon-common-extensions
```
3. Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)[^*].
4. Install Java 8: 
```
sudo apt-get update
sudo apt-get install openjdk-8-jdk
```
In case there are multiple java installations, switch to Java 8. Find the path of Java 8:
```
update-java-alternatives --list
```
Set Java 8 as default:
```
sudo update-java-alternatives --set </path/to/java/version>
```
Verify installation and check Java version:
```
java -version
```
5. Install [Gradle 6.3](https://gradle.org/install/) or higher via [SDKman](https://sdkman.io/install).
Verify Gradle installation and version:
```
gradle -v
```
6. Install [foonathan memory vendor](https://docs.px4.io/v1.12/en/dev_setup/fast-dds-installation.html):
```
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
sudo cmake --build . --target install
```
7. eigen3_cmake_module is also required, since Eigen3 is used on the transforms library:
```
sudo apt install ros-foxy-eigen3-cmake-module
```
8. Install some Python dependencies:
```
sudo apt-get update
sudo apt-get -y install python3-pip
sudo pip3 install -U empy pyros-genmsg setuptools
```
9. Clone [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) into home folder:
```
git clone https://github.com/PX4/PX4-Autopilot
```
10. Clone FASTDDS[^**] into home folder and build 
```
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.6.0 ~/FastDDS-2.6.0
cd ~/FastDDS-2.6.0
mkdir build && cd build
```
```
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
```

11. Clone Fast-RTPS-Gen into home folder and build
```
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && ./gradlew assemble \
    && sudo ./gradlew install
```


### Installation of PX4-ROS-COM
1. Create workspace for px4_ros_com_ros2
```
mkdir px4_ros_com_ros2
```
2. Clone px4_ros_com and px4_msg into src folder in workspace
```
git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com
git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
```
3. Build
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```

Verify the installation
---------------------
1. Build the workspace
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```
2. Run Gazebo with PX4 SITL
```
cd ~/PX4-Autopilot
make px4_sitl_rtps gazebo
```
3. Start micrortps_agent
```
source ~/px4_ros_com_ros2/install/setup.bash
micrortps_agent start -t UDP
```
4. Start a listener
```
source ~/px4_ros_com_ros2/install/setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
Additional Information
---------------------
### To add a publisher/subscriber message to/fro PX4 and ROS2
1. Make changes to the file "urtps_bridge_topics.yaml" in the folder ~/PX4-Autopilot/msg/tools
2. Run the code in the terminal at the base folder location 
```
~/PX4-Autopilot/msg/tools python3 uorb_to_ros_urtps_topics.py -i urtps_bridge_topics.yaml -o ~/px4_ros_com_ros2/src/px4_ros_com/templates/urtps_bridge_topics.yaml
```
3. Clean all bash by running the file at the base folder location
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts 
source clean_all.bash
```
4. Build the workspace again
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```

### To see error during compilation of PX4-ROS-COM-ROS2
1. Install simplescreenrecorder
````
sudo apt update
sudo apt install simplescreenrecorder
````
2. Use simplescreenrecorder to record the cmd screen while running the bash file
3. Revisit the recorded video to see the final instance before the cmd prompt crashes

FAQ / Troubleshooting
---------------------

[^*]: Ensure to source the ROS2 bash file for the dependencies/libraries.  
  Alternatively, use _echo "source opt/ros/foxy/setup.bash"_ to include source file in bash.rc
  
[^**]: Ensure that cloned FastDDS is compatible with Foonathan memory vendor. 

Error when cross compiling with Armadillo library
1. Comment out following line from Armadillo library (Location: usr/include/armadillo_bits)
````
#define ARMA_USE_WRAPPER
````
2. Modify CMAKE list
````
#define ARMA_USE_WRAPPER
````

Maintainers
---------------------
1. Tan Jun Kiat (junkiat@hotmail.com)
2. Karanjot Singh (karanjot007@gmail.com)
