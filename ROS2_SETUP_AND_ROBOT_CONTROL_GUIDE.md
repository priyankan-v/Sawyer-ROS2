# Sawyer ROS2 Setup, Validation, and Robot Control Guide

This guide covers:
- End-to-end environment setup for the migrated ROS2 stack.
- Reproducible validation steps.
- How to bring up and control the robot with the migrated interfaces.
- What is confirmed on this host vs what still requires bridge/hardware.

## 1) Scope and Reality Check

The migration is now functionally validated for ROS2 package build/install, launch parsing, script entrypoints, and host-side runtime error handling.

Hard requirement for real robot control:
- Sawyer remains ROS1-native at the robot master layer.
- You need ROS1 + ROS2 bridge/gateway to command hardware from ROS2 nodes.

If bridge/hardware are not present, scripts should fail cleanly (timeouts/missing topics) instead of crashing.

## 2) Host Prerequisites

### 2.1 Operating system and ROS
- Ubuntu with ROS2 Humble installed.
- Python 3.10 (ROS2 Humble default).

### 2.2 ROS2 packages/tools
Install these on the ROS2 workstation:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions
```

Note:
- `joint_state_publisher` is optional for headless bring-up, but needed for the optional joint-state-publisher launch path.

### 2.3 ROS1 + bridge prerequisites (bridge host)
Install on the bridge host (or single machine configured for dual distro):
- ROS1 distro compatible with Sawyer setup.
- ROS2 Humble.
- `ros1_bridge` and sourced ROS1/ROS2 environments.
- ROS1 tools: `roscore`, `rostopic`.

Bridge smoke script in this repo:
- `tools/bridge/ros1_ros2_bridge_smoke.sh`

## 3) Workspace Setup

From repository root:

```bash
cd /home/priyankan/Desktop/Sawyer
source /opt/ros/humble/setup.bash
```

Important:
- Full `colcon build` at workspace root can fail because ROS1/catkin meta packages are still present.
- Build the migrated ROS2 package set explicitly.

## 4) Build the Migrated ROS2 Set

```bash
cd /home/priyankan/Desktop/Sawyer
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  intera_core_msgs \
  intera_motion_msgs \
  intera_tools_description \
  sawyer_description \
  intera_interface \
  intera_examples
```

Then source overlay:

```bash
source /home/priyankan/Desktop/Sawyer/install/setup.bash
```

## 5) Thorough Validation Checklist

Run these in order.

### 5.1 Python syntax gate for ROS2 scripts

```bash
python3 -m py_compile $(find intera_sdk/intera_interface/scripts intera_sdk/intera_examples/scripts -maxdepth 1 -type f -name '*ros2.py' | sort)
```

Expected:
- No output means pass.

### 5.2 Build gate

```bash
colcon build --packages-select \
  intera_core_msgs intera_motion_msgs intera_tools_description \
  sawyer_description intera_interface intera_examples
```

Expected:
- All selected packages finish successfully.

### 5.3 Test gate

```bash
colcon test --packages-select \
  intera_core_msgs intera_motion_msgs intera_tools_description \
  sawyer_description intera_interface intera_examples
colcon test-result --all --verbose
```

Expected:
- If no unit tests are defined, summary can report `0 tests` with no errors/failures.

### 5.4 Launch parse gate

```bash
ros2 launch intera_examples gripper_joystick.launch.py --show-args
ros2 launch intera_examples joint_position_joystick.launch.py --show-args
ros2 launch intera_examples joint_trajectory_client.launch.py --show-args
ros2 launch intera_examples joint_trajectory_file_playback.launch.py --show-args
ros2 launch intera_interface joint_trajectory_action_server.launch.py --show-args
ros2 launch sawyer_description sawyer_description_rviz2.launch.py --show-args
```

Expected:
- All commands return argument schemas without launch exceptions.

### 5.5 Executable discovery gate

```bash
ros2 pkg executables intera_interface
ros2 pkg executables intera_examples
```

Expected ROS2 executables include:
- intera_interface: `*_ros2.py` utilities + `joint_trajectory_action_server_ros2.py`
- intera_examples: `camera_display_ros2.py`, `gripper_*_ros2.py`, `head_wobbler_ros2.py`, `joint_position_joystick_ros2.py`, `joint_trajectory_*_ros2.py`, `lights_blink_ros2.py`, `navigator_io_ros2.py`

### 5.6 Help-mode smoke for every ROS2 executable

```bash
for pkg in intera_interface intera_examples; do
  ros2 pkg executables $pkg | awk '$2 ~ /ros2\.py$/ {print $2}' | while read exe; do
    timeout 20s ros2 run $pkg $exe --help >/dev/null 2>&1
    echo "$pkg $exe exit=$?"
  done
done
```

Expected:
- Exit code 0 for normal help output.

### 5.7 Runtime smoke probes (host, no robot)

```bash
# Action server startup path
timeout 20s ros2 run intera_interface joint_trajectory_action_server_ros2.py --limb right --mode position

# Runtime examples with expected host-gated behavior
timeout 20s ros2 run intera_examples navigator_io_ros2.py
timeout 20s ros2 run intera_examples camera_display_ros2.py
timeout 20s ros2 run intera_examples lights_blink_ros2.py --light_name head_green_light

# Utility scripts with expected host-gated behavior
timeout 20s ros2 run intera_interface enable_robot_ros2.py -s
timeout 20s ros2 run intera_interface home_joints_ros2.py -n -t 2
timeout 20s ros2 run intera_interface io_config_editor_ros2.py -s /tmp/ee_config_test.json
timeout 20s ros2 run intera_interface send_urdf_fragment_ros2.py -f /tmp/does_not_exist.urdf
```

Expected:
- Clean startup or clean, explicit errors (missing topic/action/config), no unhandled tracebacks.

## 6) Bridge Bring-up and Verification

### 6.1 Run bridge smoke script

```bash
cd /home/priyankan/Desktop/Sawyer
bash tools/bridge/ros1_ros2_bridge_smoke.sh
```

If it reports missing prerequisites (`roscore`, `rostopic`), install ROS1 tooling and re-run.

### 6.2 Minimal bridge validation goals
- Confirm ROS1 master reachable from bridge host.
- Confirm at least one read-only robot state topic appears in ROS2 graph.
- Confirm ROS2 node can subscribe and receive messages from bridged state topic.

Do not begin motion testing until read-only bridge telemetry is stable.

## 7) Control Bring-up Sequence (Safety-First)

### 7.1 Description and static validation
Headless description bring-up:

```bash
ros2 launch sawyer_description sawyer_description_rviz2.launch.py start_rviz:=false start_joint_state_publisher:=false
```

### 7.2 Start trajectory action server

```bash
ros2 launch intera_interface joint_trajectory_action_server.launch.py limb:=right mode:=position
```

### 7.3 Query robot state and enable path

```bash
ros2 run intera_interface enable_robot_ros2.py -s
ros2 run intera_interface enable_robot_ros2.py -e
```

### 7.4 Control examples (after bridge + hardware ready)

Joint trajectory client:

```bash
ros2 launch intera_examples joint_trajectory_client.launch.py limb:=right mode:=position
```

File playback:

```bash
ros2 launch intera_examples joint_trajectory_file_playback.launch.py file_path:=/absolute/path/to/trajectory.csv loops:=1
```

Joystick position control:

```bash
ros2 launch intera_examples joint_position_joystick.launch.py joystick:=xbox dev:=/dev/input/js0 limb:=right auto_enable:=false
```

Gripper joystick:

```bash
ros2 launch intera_examples gripper_joystick.launch.py joystick:=xbox dev:=/dev/input/js0
```

Other migrated runtime examples:

```bash
ros2 run intera_examples head_wobbler_ros2.py
ros2 run intera_examples navigator_io_ros2.py
ros2 run intera_examples camera_display_ros2.py
ros2 run intera_examples gripper_keyboard_ros2.py --no-auto-enable
ros2 run intera_examples gripper_cuff_control_ros2.py --no-lights
ros2 run intera_examples lights_blink_ros2.py --light_name head_green_light
```

## 8) Confirmation Statement

What is confirmed with high confidence:
- Migrated ROS2 package set builds successfully.
- ROS2 launch files parse and expose expected arguments.
- ROS2 executables are installed/discoverable.
- Entry points run and host-gated failure paths are explicit and stable.
- Critical action server shutdown path was tested and hardened.

What cannot be fully confirmed on this host alone:
- Real robot closed-loop behavior without ROS1/ROS2 bridge and live Sawyer topics/services/actions.
- Controlled hardware motion tests and sensor/IO parity under load.

Therefore:
- Software migration quality is strongly validated for build/runtime structure.
- Final hardware correctness confirmation still requires bridge-enabled, live robot validation.

## 9) Troubleshooting

### 9.1 `colcon build` fails with catkin package errors
Cause:
- ROS1/catkin packages in the same workspace.
Fix:
- Build only migrated ROS2 packages using `--packages-select` list in Section 4.

### 9.2 Bridge smoke reports missing `roscore`/`rostopic`
Cause:
- ROS1 tools not installed/sourced.
Fix:
- Install ROS1 tooling on bridge host and source ROS1 environment before smoke script.

### 9.3 `joint_state_publisher` missing in description launch
Cause:
- Optional package not installed.
Fix:
- Install `ros-humble-joint-state-publisher` or set `start_joint_state_publisher:=false`.

### 9.4 Camera/head/navigator/gripper warnings on workstation
Cause:
- No live robot topics/params.
Fix:
- Expected on host-only runs; verify again with active bridge + robot.

## 10) Recommended Production Validation on Hardware

Run this final acceptance sequence on the bridge-connected setup:
1. Bridge smoke script passes with no missing prerequisites.
2. `enable_robot_ros2.py -s` reports valid live state.
3. Read-only checks pass for head/navigator/camera/IO signals.
4. Gripper open/close tests pass via `gripper_keyboard_ros2.py` and `gripper_cuff_control_ros2.py`.
5. Single low-amplitude trajectory via `joint_trajectory_client_ros2.py` succeeds.
6. File playback trajectory completes with expected tolerance behavior.
7. Stop/reset/disable recovery flow validated.

When all seven pass, you can treat the migration as operational for robot control.
