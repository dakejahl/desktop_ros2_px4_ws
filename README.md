# Simulating PX4 and ROS 2 on Desktop
Follow these steps to setup your desktop environment for simulating PX4 and ROS 2 with Gazebo.

## Setup PX4
Follow the PX4 docs to configure your environment for simulation. <br>
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets <br>

## Install ROS 2
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy pyros-genmsg setuptools
```
## Setup ROS 2 workspace
```
mkdir -p ~/desktop_ros2_px4_ws/src && cd ~/desktop_ros2_px4_ws/src

```
Download packages:
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
git clone https://github.com/PX4/px4_msgs.git
git clone git@github.com:gazebosim/ros_gz.git -b humble
git clone git@github.com:BrettRD/ros-gst-bridge.git -b ros2
```
Install package dependencies:
```
cd ~/desktop_ros2_px4_ws
sudo rosdep init
rosdep install -r --from-paths src/ -i -y --rosdistro humble
sudo apt-get install -y libgflags-dev
```
Build the packages:
```
export GZ_VERSION=edifice
colcon build
```
When attempting to use the newly built packages, you must always first source this file:
```
source install/setup.bash
```
