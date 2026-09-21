#### With the local Ubuntu, connect the Ethernet cable from the Sawyer controller to the workstation PC.
#### Get the IP of the Sawyer (This version)
```bash
getent hosts 021611CP00085.local
```

#### Setup local network
```bash
sudo ip addr add 169.254.24.100/16 dev enp0s31f6

ip route get 169.254.121.3 # used as ROS_IP
```

#### Check Existing Docker containers
```bash
docker ps
```

#### If there no existing container, create one and run as,
```bash
docker run -d \
    --name sawyer_noetic_pr \
    --network host \
    --add-host=021611CP00085.local:169.254.121.3 \
    -v "$HOME/sawyer_ros2:/root/sawyer_ros2" \
    -w /root/sawyer_ros2 \
    osrf/ros:noetic-desktop-full \
    tail -f /dev/null
```
#### Stop a specific container
```bash
docker stop sawyer_noetic_pr

docker start sawyer_noetic_pr

docker rm sawyer_noetic_pr #Delete the stopped container
```

#### Enter to the 'sawyer_noetic_p' and oper bash terminal
```bash
docker exec -it sawyer_noetic_p bash
```

#### Source and Verify ros1
```bash
source /opt/ros/noetic/setup.bash

rosversion -d #Expected "noetic"

ping -c 4 169.254.121.3 

ping -c 4 021611CP00085.local

getent hosts 021611CP00085.local
```

#### 
```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.24.100
unset ROS_HOSTNAME

echo $ROS_MASTER_URI
echo $ROS_IP
```

####
```bash
rostopic list

rostopic echo -n 1 /robot/joint_states


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

####
```bash

```