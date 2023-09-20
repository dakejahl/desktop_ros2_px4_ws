## ros_gz


<!-- install dds -->
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
<!-- setup and build -->
- cd desktop_ros2_px4_ws/src
- git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
- git clone git@github.com:gazebosim/ros_gz.git --branch humble
- git clone git@github.com:BrettRD/ros-gst-bridge.git
- cd ..
- rosdep install -r --from-paths src/ -i -y --rosdistro humble
- sudo apt-get install -y libgflags-dev
- colcon build

<!-- run -->
- source install/setup.bash
