#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

import intera_interface
from intera_external_devices import getch
from intera_interface.robot_params_ros2 import RobotParamsROS2


def map_keyboard(node, limb, auto_enable=True):
    node.get_logger().info("Getting robot state...")
    try:
        rs = intera_interface.RobotEnableROS2(node=node, versioned=False)
    except OSError as exc:
        node.get_logger().error(f"Robot state is unavailable: {exc}")
        return 1
    gripper = None
    original_deadzone = None

    def clean_shutdown():
        if gripper and original_deadzone is not None:
            gripper.set_dead_zone(original_deadzone)
        node.get_logger().info("Exiting example")

    try:
        gripper = intera_interface.GripperROS2(limb + "_gripper", node=node)
    except (ValueError, OSError) as exc:
        node.get_logger().error(f"Could not detect an electric gripper attached to the robot: {exc}")
        clean_shutdown()
        return 1

    if auto_enable:
        node.get_logger().info("Enabling robot...")
        rs.enable()

    original_deadzone = gripper.get_dead_zone()
    gripper.set_dead_zone(0.001)
    node.get_logger().info(f"Gripper deadzone set to {gripper.get_dead_zone()}")

    num_steps = 8.0
    percent_delta = 1.0 / num_steps
    velocity_increment = (gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) * percent_delta
    position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta

    def offset_position(offset_pos):
        current = gripper.get_position() or 0.0
        cmd_pos = max(min(current + offset_pos, gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        node.get_logger().info(f"commanded position set to {cmd_pos} m")

    def update_velocity(offset_vel):
        current = gripper.get_cmd_velocity() or gripper.MIN_VELOCITY
        cmd_speed = max(min(current + offset_vel, gripper.MAX_VELOCITY), gripper.MIN_VELOCITY)
        gripper.set_cmd_velocity(cmd_speed)
        node.get_logger().info(f"commanded velocity set to {cmd_speed} m/s")

    bindings = {
        "r": (gripper.reboot, [], "reboot"),
        "c": (gripper.calibrate, [], "calibrate"),
        "q": (gripper.close, [], "close"),
        "o": (gripper.open, [], "open"),
        "+": (update_velocity, [velocity_increment], f"increase velocity by {percent_delta * 100}%"),
        "-": (update_velocity, [-velocity_increment], f"decrease velocity by {percent_delta * 100}%"),
        "s": (gripper.stop, [], "stop"),
        "u": (offset_position, [-position_increment], f"decrease position by {percent_delta * 100}%"),
        "i": (offset_position, [position_increment], f"increase position by {percent_delta * 100}%"),
    }

    node.get_logger().info("Controlling grippers. Press ? for help, Esc to quit.")
    done = False
    while not done and node.context.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        c = getch(timeout=0.01)
        if not c:
            continue

        if c in ["\x1b", "\x03"]:
            done = True
        elif c in bindings:
            cmd = bindings[c]
            node.get_logger().info(f"command: {cmd[2]}")
            cmd[0](*cmd[1])
        else:
            print("key bindings:")
            print("  Esc: Quit")
            print("  ?: Help")
            for key, val in sorted(list(bindings.items()), key=lambda x: x[1][2]):
                print(f"  {key}: {val[2]}")

    clean_shutdown()
    return 0


def main():
    epilog = "See help inside the example with the '?' key for key bindings."

    rclpy.init()
    node = Node("sdk_gripper_keyboard_ros2")

    rp = RobotParamsROS2(node)
    valid_limbs = rp.get_limb_names() or ["right"]

    parser = argparse.ArgumentParser(description="RSDK Gripper Example: Keyboard Control (ROS2)", epilog=epilog)
    parser.add_argument(
        "-l",
        "--limb",
        dest="limb",
        default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the gripper keyboard example",
    )
    parser.add_argument(
        "--no-auto-enable",
        action="store_true",
        help="Do not enable robot automatically on startup",
    )
    args = parser.parse_args()

    try:
        return map_keyboard(node, args.limb, auto_enable=(not args.no_auto_enable))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
