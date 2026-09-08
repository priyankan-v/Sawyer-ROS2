# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import errno
import math
import time


def wait_for_ros2(test, *, node, timeout=10.0, raise_on_error=True, rate=100.0, timeout_msg="timeout expired", body=None):
    """Wait until a condition evaluates true while spinning a ROS 2 node.

    Args:
        test: Zero-arg callable that returns truthy when complete.
        node: rclpy.node.Node used for shutdown state and spinning.
        timeout: Seconds to wait. Negative or inf means no timeout.
        raise_on_error: Raise OSError on shutdown/timeout when true.
        rate: Loop frequency in Hz.
        timeout_msg: Message used in timeout exception.
        body: Optional callable invoked each loop iteration.
    """
    period = 1.0 / rate if rate > 0.0 else 0.01
    start = time.monotonic()
    no_timeout = timeout < 0.0 or math.isinf(timeout)

    while not test():
        if not node.context.ok():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS 2 shutdown")
            return False

        elapsed = time.monotonic() - start
        if (not no_timeout) and elapsed >= timeout:
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            return False

        if callable(body):
            body()

        # Progress callbacks and subscriptions while waiting.
        import rclpy

        rclpy.spin_once(node, timeout_sec=period)

    return True
