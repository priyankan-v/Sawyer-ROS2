# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import collections
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header

from intera_core_msgs.msg import (
    CollisionDetectionState,
    EndpointState,
    EndpointStates,
    JointCommand,
)
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK
from intera_dataflow.wait_for_ros2 import wait_for_ros2

from . import settings
from .robot_params_ros2 import RobotParamsROS2


class LimbROS2:
    """ROS 2 interface class for a limb on Intera robots."""

    Point = collections.namedtuple("Point", ["x", "y", "z"])
    Quaternion = collections.namedtuple("Quaternion", ["x", "y", "z", "w"])

    def __init__(
        self,
        limb="right",
        synchronous_pub=False,
        node=None,
        require_joint_states=True,
        require_endpoint_state=True,
        require_tip_states=True,
    ):
        del synchronous_pub  # ROS 2 publishers are always asynchronous.

        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node(f"intera_limb_{limb}_ros2")
            self._owns_node = True

        self._node = node

        params = RobotParamsROS2(self._node)
        limb_names = params.get_limb_names()
        if limb_names and limb not in limb_names:
            raise ValueError(f"Cannot detect limb {limb}; valid limbs are {limb_names}")

        joint_names = params.get_joint_names(limb)
        if not joint_names:
            joint_names = [f"{limb}_j{i}" for i in range(7)]
            self._node.get_logger().warning(
                "Falling back to default 7-DOF joint names. Declare robot_config parameters for full fidelity."
            )

        self.name = limb
        self._joint_names = list(joint_names)
        self._joint_angle = {}
        self._joint_velocity = {}
        self._joint_effort = {}
        self._cartesian_pose = {}
        self._cartesian_velocity = {}
        self._cartesian_effort = {}
        self._collision_state = False
        self._tip_states = None

        ns = f"/robot/limb/{limb}/"

        self._command_msg = JointCommand()

        self._pub_speed_ratio = self._node.create_publisher(Float64, ns + "set_speed_ratio", 10)
        self._pub_joint_cmd = self._node.create_publisher(JointCommand, ns + "joint_command", 10)
        self._pub_joint_cmd_timeout = self._node.create_publisher(Float64, ns + "joint_command_timeout", 10)

        self._node.create_subscription(EndpointState, ns + "endpoint_state", self._on_endpoint_states, 10)
        self._node.create_subscription(EndpointStates, ns + "tip_states", self._on_tip_states, 10)
        self._node.create_subscription(
            CollisionDetectionState,
            ns + "collision_detection_state",
            self._on_collision_state,
            10,
        )
        self._node.create_subscription(JointState, "/robot/joint_states", self._on_joint_states, 20)

        ns_pkn = f"/ExternalTools/{limb}/PositionKinematicsNode/"
        self._ik_client = self._node.create_client(SolvePositionIK, ns_pkn + "IKService")
        self._fk_client = self._node.create_client(SolvePositionFK, ns_pkn + "FKService")

        if require_joint_states:
            err_msg = f"{self.name.capitalize()} limb init failed to get current joint_states"
            wait_for_ros2(lambda: len(self._joint_angle) > 0, node=self._node, timeout=5.0, timeout_msg=err_msg)

        if require_endpoint_state:
            err_msg = f"{self.name.capitalize()} limb init failed to get current endpoint_state"
            wait_for_ros2(lambda: len(self._cartesian_pose) > 0, node=self._node, timeout=5.0, timeout_msg=err_msg)

        if require_tip_states:
            err_msg = f"{self.name.capitalize()} limb init failed to get current tip_states"
            wait_for_ros2(lambda: self._tip_states is not None, node=self._node, timeout=5.0, timeout_msg=err_msg)

    def destroy(self):
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx] if idx < len(msg.position) else 0.0
                self._joint_velocity[name] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                self._joint_effort[name] = msg.effort[idx] if idx < len(msg.effort) else 0.0

    def _on_endpoint_states(self, msg):
        self._cartesian_pose = {
            "position": self.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            "orientation": self.Quaternion(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        }
        self._cartesian_velocity = {
            "linear": self.Point(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
            "angular": self.Point(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z),
        }
        self._cartesian_effort = {
            "force": self.Point(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z),
            "torque": self.Point(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z),
        }

    def _on_tip_states(self, msg):
        self._tip_states = deepcopy(msg)

    def _on_collision_state(self, msg):
        self._collision_state = bool(msg.collision_state)

    def has_collided(self):
        return self._collision_state

    def joint_names(self):
        return list(self._joint_names)

    def joint_angle(self, joint):
        return self._joint_angle[joint]

    def joint_angles(self):
        return deepcopy(self._joint_angle)

    def joint_ordered_angles(self):
        return [self._joint_angle[name] for name in self._joint_names]

    def joint_velocity(self, joint):
        return self._joint_velocity[joint]

    def joint_velocities(self):
        return deepcopy(self._joint_velocity)

    def joint_effort(self, joint):
        return self._joint_effort[joint]

    def joint_efforts(self):
        return deepcopy(self._joint_effort)

    def endpoint_pose(self):
        return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self):
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        return deepcopy(self._cartesian_effort)

    def tip_state(self, tip_name):
        try:
            return deepcopy(self._tip_states.states[self._tip_states.names.index(tip_name)])
        except (ValueError, AttributeError):
            return None

    def _publish_float(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def set_command_timeout(self, timeout):
        self._publish_float(self._pub_joint_cmd_timeout, timeout)

    def exit_control_mode(self, timeout=0.2):
        self.set_command_timeout(timeout)
        self.set_joint_positions(self.joint_angles())

    def set_joint_position_speed(self, speed=0.3):
        self._publish_float(self._pub_speed_ratio, speed)

    def set_joint_trajectory(self, names, positions, velocities, accelerations):
        self._command_msg.names = list(names)
        self._command_msg.position = list(positions)
        self._command_msg.velocity = list(velocities)
        self._command_msg.acceleration = list(accelerations)
        self._command_msg.mode = JointCommand.TRAJECTORY_MODE
        self._command_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_positions(self, positions):
        self._command_msg.names = list(positions.keys())
        self._command_msg.position = list(positions.values())
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._command_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        self._command_msg.names = list(velocities.keys())
        self._command_msg.velocity = list(velocities.values())
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._command_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_torques(self, torques):
        self._command_msg.names = list(torques.keys())
        self._command_msg.effort = list(torques.values())
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._command_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_neutral(self, timeout=15.0, speed=0.3):
        param_name = f"named_poses.{self.name}.poses.neutral"
        if not self._node.has_parameter(param_name):
            self._node.get_logger().error(f"Get neutral pose failed for limb {self.name}")
            return False
        neutral_pose = self._node.get_parameter(param_name).value
        angles = dict(zip(self.joint_names(), neutral_pose))
        self.set_joint_position_speed(speed)
        return self.move_to_joint_positions(angles, timeout=timeout)

    def move_to_joint_positions(self, positions, timeout=15.0, threshold=settings.JOINT_ANGLE_TOLERANCE, test=None):
        fail_msg = f"{self.name.capitalize()} limb failed to reach commanded joint positions"

        def complete():
            if self.has_collided():
                self._node.get_logger().error(f"Collision detected. {fail_msg}")
                return True
            if callable(test) and test():
                return True
            for joint, angle in positions.items():
                if joint in self._joint_angle and abs(angle - self._joint_angle[joint]) >= threshold:
                    return False
            return True

        self.set_joint_positions(positions)
        wait_for_ros2(
            complete,
            node=self._node,
            timeout=timeout,
            timeout_msg=fail_msg,
            raise_on_error=False,
            rate=100.0,
            body=lambda: self.set_joint_positions(positions),
        )
        return not self.has_collided()

    def _call_service(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error("Required service is unavailable")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            self._node.get_logger().error("Service call timed out or failed")
            return None
        return future.result()

    def ik_request(
        self,
        pose,
        end_point="right_hand",
        joint_seed=None,
        nullspace_goal=None,
        nullspace_gain=0.4,
        allow_collision=False,
    ):
        if not isinstance(pose, Pose):
            self._node.get_logger().error("pose is not geometry_msgs.msg.Pose")
            return False

        req = SolvePositionIK.Request()
        req.pose_stamp.append(PoseStamped(header=Header(stamp=self._node.get_clock().now().to_msg(), frame_id="base"), pose=pose))
        req.tip_names.append(end_point)

        if joint_seed is not None:
            req.seed_mode = req.SEED_USER
            seed = JointState()
            seed.name = list(joint_seed.keys())
            seed.position = list(joint_seed.values())
            req.seed_angles.append(seed)

        if nullspace_goal is not None:
            req.use_nullspace_goal.append(True)
            goal = JointState()
            goal.name = list(nullspace_goal.keys())
            goal.position = list(nullspace_goal.values())
            req.nullspace_goal.append(goal)
            req.nullspace_gain.append(float(nullspace_gain))

        resp = self._call_service(self._ik_client, req)
        if resp is None or not resp.result_type:
            return False

        result_type = resp.result_type[0]
        if result_type > 0 or (allow_collision and result_type == resp.IK_IN_COLLISION):
            return dict(zip(resp.joints[0].name, resp.joints[0].position))

        self._node.get_logger().error("INVALID POSE - No valid IK solution found")
        return False

    def fk_request(self, joint_angles, end_point="right_hand"):
        req = SolvePositionFK.Request()
        joints = JointState()
        joints.name = list(joint_angles.keys())
        joints.position = list(joint_angles.values())
        req.configuration.append(joints)
        req.tip_names.append(end_point)
        return self._call_service(self._fk_client, req)

    def joint_angles_to_cartesian_pose(self, joint_angles, end_point="right_hand"):
        resp = self.fk_request(joint_angles, end_point=end_point)
        if resp is None:
            return None
        if not resp.is_valid or not resp.is_valid[0]:
            self._node.get_logger().info("INVALID JOINTS - No Cartesian solution found")
            return None
        return resp.pose_stamp[0].pose

    def fk_request_in_collision(self, joint_angles, end_point="right_hand"):
        resp = self.fk_request(joint_angles, end_point=end_point)
        if resp is None:
            return True
        if not resp.is_valid or not resp.is_valid[0]:
            return True
        return bool(resp.in_collision[0])
