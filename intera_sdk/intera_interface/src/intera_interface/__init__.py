# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

try:
    from .camera import Cameras
except Exception:
    Cameras = None

try:
    from .digital_io import DigitalIO
except Exception:
    DigitalIO = None

try:
    from .gripper import Gripper
except Exception:
    Gripper = None

try:
    from .clicksmart_plate import SimpleClickSmartGripper
except Exception:
    SimpleClickSmartGripper = None

try:
    from .gripper_factory import get_current_gripper_interface
except Exception:
    get_current_gripper_interface = None

try:
    from .cuff import Cuff
except Exception:
    Cuff = None

try:
    from .head import Head
except Exception:
    Head = None

try:
    from .head_display import HeadDisplay
except Exception:
    HeadDisplay = None

try:
    from .joint_limits import JointLimits
except Exception:
    JointLimits = None

try:
    from .lights import Lights
except Exception:
    Lights = None

try:
    from .limb import Limb
except Exception:
    Limb = None

try:
    from .navigator import Navigator
except Exception:
    Navigator = None

try:
    from .robot_enable import RobotEnable
except Exception:
    RobotEnable = None

try:
    from .robot_params import RobotParams
except Exception:
    RobotParams = None

from .head_ros2 import HeadROS2
from .gripper_ros2 import GripperROS2
from .cuff_ros2 import CuffROS2
from .lights_ros2 import LightsROS2
from .limb_ros2 import LimbROS2
from .navigator_ros2 import NavigatorROS2
from .robot_enable_ros2 import RobotEnableROS2
from .camera_ros2 import CamerasROS2
from .settings import (
    JOINT_ANGLE_TOLERANCE,
    HEAD_PAN_ANGLE_TOLERANCE,
    SDK_VERSION,
    CHECK_VERSION,
    VERSIONS_SDK2ROBOT,
    VERSIONS_SDK2GRIPPER,
)
