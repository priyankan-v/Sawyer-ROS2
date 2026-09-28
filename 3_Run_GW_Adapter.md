## Use three terminals.

```bash
history -c
```
Clarify the IP addresses
```bash
getent hosts 021611CP00085.local

ip route get 169.254.121.3
```

## Terminal 1 — ROS 1 Docker
```bash
docker exec -it sawyer_noetic_pr bash
```
```bash
echo $ROS_MASTER_URI
echo $ROS_IP
```

If this is a new shell, you may need to export them again:

```bash
export ROS_MASTER_URI=http://021611CP00085.local:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
```

```bash
cd /root/sawyer_ros2/ros1_ws

source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

```bash
rostopic echo -n 1 /robot/joint_states
```

```bash
rosrun sawyer_ros1_adapter joint_state_adapter.py
```
It should stop here: 
'Waiting for ROS 2 gateway on 127.0.0.1:5005...'


## Terminal 2 on the host

```bash
source /opt/ros/humble/setup.bash

source ~/sawyer_ros2/ros2_ws/install/setup.bash

ros2 run sawyer_ros2_gateway joint_state_gateway
```

## Terminal 3 on the host

```bash
source /opt/ros/humble/setup.bash
source ~/sawyer_ros2/ros2_ws/install/setup.bash
ros2 topic list
```

```bash
ros2 topic echo /joint_states --once
```

```bash
ros2 topic hz /joint_states
```

```bash
ros2 topic info /joint_states
```

```bash

```

```bash

```

```bash

```


```bash

```
