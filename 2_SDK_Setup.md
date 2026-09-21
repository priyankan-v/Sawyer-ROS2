# Install the Sawyer Intera SDK (ROS 1 Noetic)

Complete [Docker setup](1_Docker_Setup.md) first. The commands below assume its
`sawyer_noetic_pr` container is running, this repository is mounted at
`/root/sawyer_ros2`, and the container can reach the Sawyer controller. Run all
commands **inside the container** unless a step says otherwise. This builds the
ROS 1 SDK workspace; it does not install the SDK into a ROS 2 workspace.

## 1. Enter the container

On the Ubuntu host:

```bash
docker start sawyer_noetic_pr  # Only needed if the container is stopped
docker exec -it sawyer_noetic_pr bash
```

## 2. Link the SDK packages into a catkin workspace

The three SDK repositories already live at the root of this checkout. Link
their catkin packages into `ros1_ws/src` so there is only one copy of each
package. From inside the container:

```bash
cd /root/sawyer_ros2
mkdir -p ros1_ws/src

for package in intera_common/* intera_sdk/* sawyer_robot/*; do
    [ -f "$package/package.xml" ] || continue
    link="ros1_ws/src/${package##*/}"
    if [ ! -e "$link" ] && [ ! -L "$link" ]; then
        ln -s "../../$package" "$link"
    fi
done

ls -l ros1_ws/src
```

The result should include `intera_core_msgs`, `intera_motion_msgs`,
`intera_interface`, `intera_examples`, `sawyer_description`, and the three
metapackages. If a name already exists in `src`, check that it points to the
matching package in this checkout before building.

## 3. Install dependencies

```bash
source /opt/ros/noetic/setup.bash
apt update
apt install -y \
    python3-rosdep \
    build-essential \
    ros-noetic-joystick-drivers \
    ros-noetic-rospy-message-converter

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
rosdep update

cd /root/sawyer_ros2/ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic \
    --skip-keys "joystick_drivers rospy_message_converter" -r -y
```

The packages declare `joystick_drivers` and `rospy_message_converter`, but
rosdep has no Noetic rules for these keys in this setup. The explicit
`apt install` supplies both; `--skip-keys` only skips rosdep's lookup for them.
Rosdep installs the remaining declared dependencies.

## 4. Build and check the workspace

```bash
cd /root/sawyer_ros2/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake
source devel/setup.bash

rospack find intera_core_msgs
rospack find intera_motion_msgs
rospack find intera_interface
rospack find intera_examples
rospack find sawyer_description
rosmsg show intera_core_msgs/JointCommand
```

The `rospack` commands should print paths under `ros1_ws/src`; `rosmsg`
should print the `JointCommand` fields. These checks do not require a live
robot.

## 5. Configure the robot connection

Copy the SDK's environment script to the catkin workspace root **after** the
build. The script expects `devel/setup.bash` there.

```bash
cd /root/sawyer_ros2
cp intera_sdk/intera.sh ros1_ws/intera.sh
chmod +x ros1_ws/intera.sh
nano ros1_ws/intera.sh
```

Set these three variables near the top of the copied script, using the actual
robot hostname and the workstation address on the robot-facing network:

```bash
robot_hostname="021611CP00085.local"
your_ip="169.254.24.100"
ros_version="noetic"
```

The example values match [Docker setup](1_Docker_Setup.md). If the host's
address or robot address differs, use the address reported by
`ip route get <robot-ip>` for `your_ip` and make sure `robot_hostname`
resolves inside the container. The script sets `ROS_MASTER_URI` to the robot
and `ROS_IP` to the workstation address.

## 6. Open an SDK shell and verify connectivity

Run the script from the workspace root. It opens a new shell with the SDK and
robot environment loaded; use `exit` to return to the original shell.

```bash
cd /root/sawyer_ros2/ros1_ws
./intera.sh

echo "$ROS_MASTER_URI"
echo "$ROS_IP"
rostopic list
rostopic echo -n 1 /robot/joint_states
```

The final two commands require a powered, network-reachable robot with its
ROS master running. Run `./intera.sh` again for each new container shell in
which you use the SDK.
