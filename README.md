# Simulating PX4 and ROS 2 on Desktop
Follow these steps to setup your desktop environment for simulating PX4 and ROS 2 with Gazebo. Instructions below show you how to
setup a video stream from a simulation

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
git clone https://github.com/gazebosim/ros_gz.git -b humble
git clone https://github.com/BrettRD/ros-gst-bridge.git -b ros2
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
export GZ_VERSION=garden
colcon build
```
When attempting to use the newly built packages, you must always first source this file:
```
source install/setup.bash
```
## Simulating an X500 with an OAK-D Lite depth camera
Build and start the simulation:
```
make px4_sitl
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_GZ_WORLD=default ./build/px4_sitl_default/bin/px4
```
Add an alias to make it as simple as `px4_sim`
```
mkdir -p ~/.bash_aliases
echo "alias px4_sim=\"PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth PX4_GZ_MODEL_POSE=\"0,0,0,0,0,0\" PX4_GZ_WORLD=default ./build/px4_sitl_default/bin/px4\"" >> ~/.bash_aliases
```
You can see topics in gazebo, notice the camera and odometry topics
```
gz topic --list
```
> /camera <br>
> /camera_info <br>
> /clock <br>
> /depth_camera <br>
> /depth_camera/points <br>
> /gazebo/resource_paths <br>
> /gui/camera/pose <br>
> /model/x500_depth_0/odometry <br>
> /model/x500_depth_0/odometry_with_covariance <br>
> /model/x500_depth_0/pose <br>

`/model/x500_depth_0/odometry` comes from the OdometryPublisher in the **models/x500_depth/model.sdf**. This is already
fed to PX4 via the GZBridge, alternatively we should be able to supply it from a ros2 node as well.


### Video streaming
We can get a video stream from the simulation by bridging the gazebo `/camera` topic to ros2 using the [ros_gz_image](https://github.com/gazebosim/ros_gz#integration-between-ros-and-gazebo) package.
1. Bridge gazebo images to ros2
```
source ~/desktop_ros2_px4_ws/install/setup.bash
ros2 run ros_gz_image image_bridge /camera
```
2. Launch gstreamer using rosimagesrc from the [ros-gst-bridge](https://github.com/BrettRD/ros-gst-bridge) package.
```
~/desktop_ros2_px4_ws/
source install/setup.bash
gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="/camera" ! queue max-size-buffers=1 ! videoconvert ! "video/x-raw,format=I420" ! x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5600 sync=false
```
You can view the video in QGC or display with a gstreamer client
```
gst-launch-1.0 udpsrc port=5600 ! application/x-rtp ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink

```