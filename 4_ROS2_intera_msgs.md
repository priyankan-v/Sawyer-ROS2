## On the host

```bash
cd ~/sawyer_ros2/ros2_ws/src

source /opt/ros/humble/setup.bash

ros2 pkg create \
    --build-type ament_cmake \
    intera_core_msgs
```

```bash
cd intera_core_msgs
mkdir -p msg
```

```bash
cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/EndpointState.msg msg/

cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/RobotAssemblyState.msg msg/

cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/JointCommand.msg msg/
```

```bash

```

```bash

```