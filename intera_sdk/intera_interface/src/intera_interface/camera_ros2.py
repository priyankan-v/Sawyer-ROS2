# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from intera_core_msgs.msg import IONodeConfiguration
from intera_dataflow.wait_for_ros2 import wait_for_ros2
from intera_io.io_interface_ros2 import IODeviceInterfaceROS2

from .robot_params_ros2 import RobotParamsROS2


class CamerasROS2:
    """ROS 2 interface to robot cameras."""

    def __init__(self, node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_cameras_ros2")
            self._owns_node = True

        self._node = node
        self._node_config = None
        self._image_subscriptions = []
        self.cameras_io = {}

        self._node_config_sub = self._node.create_subscription(
            IONodeConfiguration,
            "/io/internal_camera/config",
            self._node_config_cb,
            10,
        )

        camera_param_dict = RobotParamsROS2(self._node).get_camera_details()
        camera_list = list(camera_param_dict.keys())
        if not camera_list:
            self._node.get_logger().error("Camera list is empty")
            return

        wait_for_ros2(
            lambda: self._node_config is not None,
            node=self._node,
            timeout=5.0,
            timeout_msg="Failed to connect to camera node and retrieve configuration",
            raise_on_error=False,
        )

        cameras_to_load = list(self._get_camera_launch_config().keys()) if self._node_config else camera_list

        camera_capabilities = {
            "mono": ["cognex"],
            "color": ["ienso_ethernet"],
            "auto_exposure": ["ienso_ethernet"],
            "auto_gain": ["ienso_ethernet"],
        }

        for camera in camera_list:
            camera_type = camera_param_dict[camera].get("cameraType", camera_param_dict[camera].get("camera_type", ""))
            try:
                interface = IODeviceInterfaceROS2(
                    node=self._node,
                    node_name="internal_camera",
                    dev_name=camera,
                    require_config=False,
                    require_state=False,
                )
                self.cameras_io[camera] = {
                    "interface": interface,
                    "is_color": (camera_type in camera_capabilities["color"]),
                    "has_auto_exposure": (camera_type in camera_capabilities["auto_exposure"]),
                    "has_auto_gain": (camera_type in camera_capabilities["auto_gain"]),
                }
            except OSError:
                if camera not in cameras_to_load:
                    self._node.get_logger().warning(
                        f"Expected camera ({camera}) is not configured in this run configuration"
                    )
                else:
                    self._node.get_logger().error(f"Could not find expected camera ({camera}) for this robot")

    def destroy(self):
        self._image_subscriptions.clear()
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _node_config_cb(self, msg):
        self._node_config = msg

    def _get_camera_launch_config(self):
        if self._node_config is None:
            return {}

        plugins = self._node_config.plugins
        cameras_params = {}
        for plugin in plugins:
            plugin_config = json.loads(plugin.config)
            if "params" in plugin_config and "cameras" in plugin_config["params"]:
                for camera in plugin_config["params"]["cameras"]:
                    cameras_params[camera["name"]] = camera
        return cameras_params

    def _camera_streaming_status(self, camera_name):
        return self.cameras_io[camera_name]["interface"].get_signal_value("camera_streaming")

    def _get_signal_status(self, camera_name, signal_name):
        for signal in self.cameras_io[camera_name]["interface"].state.signals:
            if signal.name == signal_name:
                return signal.status
        return None

    def list_cameras(self):
        return list(self.cameras_io.keys())

    def verify_camera_exists(self, camera_name):
        if camera_name not in self.list_cameras():
            cameras = ", ".join(self.list_cameras())
            self._node.get_logger().error(f"{camera_name} not in detected cameras: {cameras}")
            return False
        return True

    def is_camera_streaming(self, camera_name):
        if self.verify_camera_exists(camera_name):
            return self._camera_streaming_status(camera_name)
        return False

    def set_callback(self, camera_name, callback, callback_args=None, queue_size=1, buff_size=2**23, rectify_image=True):
        del queue_size
        del buff_size
        if not self.verify_camera_exists(camera_name):
            return

        if rectify_image:
            image_string = "image_rect_color" if self.cameras_io[camera_name]["is_color"] else "image_rect"
        else:
            image_string = "image_raw"

        topic = "/".join(["/io/internal_camera", camera_name, image_string])
        if callback_args is None:
            sub = self._node.create_subscription(Image, topic, callback, 10)
        else:
            sub = self._node.create_subscription(
                Image,
                topic,
                lambda msg: callback(msg, callback_args),
                10,
            )
        self._image_subscriptions.append(sub)

    def start_streaming(self, camera_name):
        if not self.verify_camera_exists(camera_name):
            return False

        if self._camera_streaming_status(camera_name):
            return True

        other_cameras = list(set(self.list_cameras()) - set([camera_name]))
        for other_camera in other_cameras:
            if self._camera_streaming_status(other_camera):
                self.cameras_io[other_camera]["interface"].set_signal_value("camera_streaming", False, signal_type="bool")

        self.cameras_io[camera_name]["interface"].set_signal_value("camera_streaming", True, signal_type="bool")
        return True

    def stop_streaming(self, camera_name):
        if not self.verify_camera_exists(camera_name):
            return False

        if not self._camera_streaming_status(camera_name):
            return True

        self.cameras_io[camera_name]["interface"].set_signal_value("camera_streaming", False, signal_type="bool")
        return True

    def get_exposure(self, camera_name):
        if self.verify_camera_exists(camera_name):
            return self.cameras_io[camera_name]["interface"].get_signal_value("set_exposure")
        return None

    def get_gain(self, camera_name):
        if self.verify_camera_exists(camera_name):
            return self.cameras_io[camera_name]["interface"].get_signal_value("set_gain")
        return None

    def set_exposure(self, camera_name, exposure):
        if not self.verify_camera_exists(camera_name):
            return False

        if exposure == -1 and not self.cameras_io[camera_name]["has_auto_exposure"]:
            self._node.get_logger().error(f"Camera ({camera_name}) does not support auto-exposure")
            return False

        self.cameras_io[camera_name]["interface"].set_signal_value("set_exposure", exposure)
        status = self._get_signal_status(camera_name, "set_exposure")
        if status is None:
            return True
        return status.tag == "ready"

    def set_gain(self, camera_name, gain):
        if not self.verify_camera_exists(camera_name):
            return False

        if gain == -1 and not self.cameras_io[camera_name]["has_auto_gain"]:
            self._node.get_logger().error(f"Camera ({camera_name}) does not support auto-gain")
            return False

        self.cameras_io[camera_name]["interface"].set_signal_value("set_gain", gain)
        status = self._get_signal_status(camera_name, "set_gain")
        if status is None:
            return True
        return status.tag == "ready"

    def set_cognex_strobe(self, value):
        try:
            self.cameras_io["right_hand_camera"]["interface"].set_signal_value("set_strobe", bool(value), signal_type="bool")
            return True
        except KeyError:
            self._node.get_logger().error("Cannot find Cognex camera with the name right_hand_camera")
            return False
