### Create a folder and link files
```bash
cd /root/sawyer_ros2

mkdir -p ros1_ws/src


ln -s ../../intera_common/intera_common \
    ros1_ws/src/intera_common

ln -s ../../intera_common/intera_core_msgs \
    ros1_ws/src/intera_core_msgs

ln -s ../../intera_common/intera_motion_msgs \
    ros1_ws/src/intera_motion_msgs

ln -s ../../intera_common/intera_tools_description \
    ros1_ws/src/intera_tools_description

ln -s ../../intera_sdk/intera_interface \
    ros1_ws/src/intera_interface

ln -s ../../intera_sdk/intera_examples \
    ros1_ws/src/intera_examples

ln -s ../../intera_sdk/intera_sdk \
    ros1_ws/src/intera_sdk

ln -s ../../sawyer_robot/sawyer_description \
    ros1_ws/src/sawyer_description

ln -s ../../sawyer_robot/sawyer_robot \
    ros1_ws/src/sawyer_robot

ls -l ros1_ws/src #Verify
```

####
```bash
source /opt/ros/noetic/setup.bash

apt update
apt install -y \
    python3-rosdep \
    python3-catkin-tools \
    python3-wstool \
    python3-rosinstall \
    build-essential

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi

apt-cache policy ros-noetic-message-converter

apt install -y ros-noetic-message-converter

cd /root/sawyer_ros2/ros1_ws/src

rm intera_examples
rm intera_sdk
rm sawyer_robot

cd /root/sawyer_ros2/ros1_ws

source /opt/ros/noetic/setup.bash

rosdep update

cd /root/sawyer_ros2/ros1_ws

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y \
    --rosdistro noetic

apt update

source /opt/ros/noetic/setup.bash

catkin_make

source devel/setup.bash

rospack find intera_core_msgs
rospack find intera_motion_msgs
rospack find intera_interface
rospack find sawyer_description

rosmsg show intera_core_msgs/JointCommand

cd /root/sawyer_ros2/ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make

source devel/setup.bash

cd /root/sawyer_ros2

cp intera_sdk/intera.sh ros1_ws/intera.sh
chmod +x ros1_ws/intera.sh

nano ros1_ws/intera.sh
```

####
```bash

```

####
```bash

```

####
```bash

```

####
```bash

```

####
```bash

```

####
```bash

```