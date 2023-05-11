#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver_msgs.msg import Status

from collections import defaultdict
import itertools
import math


class StatusToTwist(object):
    def __init__(self, node):
        self._node = node
        self._node.declare_parameter("stamped", False)
        self._node.declare_parameter("frame_id", "base_link")

        self._logger = self._node.get_logger()

        self._stamped = self._node.get_parameter("stamped").value
        self._frame_id = self._node.get_parameter("frame_id").value
        if self._stamped:
            self._cls = TwistStamped
        else:
            self._cls = Twist

        # Automatically create missing keys in a dict
        def make_defaultdict():
            return defaultdict(make_defaultdict)

        param_dict = make_defaultdict()
        param_types = ["inputs", "scales"]
        param_categories = ["angular", "linear"]
        param_axis = ["x", "y", "z"]
        for t, c, a in itertools.product(param_types, param_categories, param_axis):
            param_name = "{}.{}.{}".format(t, c, a)
            if t == "inputs":
                self._node.declare_parameter(param_name, "")
            elif t == "scales":
                self._node.declare_parameter(param_name, 0.0)

            param_value = self._node.get_parameter(param_name).value
            if param_value not in (None, ""):
                param_dict[t][c][a] = param_value

        # Convert back to dict (in case a non-existent key is accessed later)
        self._inputs = {k: dict(v) for k, v in param_dict["inputs"].items()}
        self._scales = {k: dict(v) for k, v in param_dict["scales"].items()}

        if self._inputs == {}:
            msg = "inputs parameter is not specified: not doing anything"
            self._logger.warning(msg)

        self._attrs = []
        for attr in Status.__slots__:
            # ROS2 message slots have a prepended underscore
            if (
                attr.startswith("_axis_")
                or attr.startswith("_button_")
                or attr.startswith("_touch0")
            ):
                self._attrs.append(attr[1:])  # get rid of the prepended underscore
        self._pub = self._node.create_publisher(self._cls, "cmd_vel", 0)
        self._sub = self._node.create_subscription(
            Status, "ds4_driver/status", self.cb_status, 0
        )
        self._init_run = True
        self._height = -1.0  # store height for further implementation
        self._x_locked = False
        self._x_locked_value = 0.0
        self._triangle_pressed = False

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        if self._init_run:
            self._init_touch0_id = getattr(msg, "touch0").id
            self._init_run = False

        input_vals = {}
        for attr in self._attrs:
            if attr.startswith("touch0"):
                input_vals[attr] = getattr(msg, attr)
                if input_vals[attr].id == self._init_touch0_id:
                    input_vals[attr].x = 0.0
                    input_vals[attr].y = 1.0
            else:
                input_vals[attr] = getattr(msg, attr)

        to_pub = self._cls()
        if self._stamped:
            to_pub.header.stamp = self._node.get_clock().now().to_msg()
            to_pub.header.frame_id = self._frame_id
            twist = to_pub.twist
        else:
            twist = to_pub

        for vel_type in self._inputs.keys():
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k, 1.0)
                if scale is None:
                    scale = 1.0
                try:
                    if expr == "l2_r2_heightfunction":
                        val = self.l2_r2_heightfunction(input_vals)
                    elif k == "x":
                        val = eval(expr, {}, input_vals)
                        if not self._triangle_pressed and input_vals["button_triangle"]:
                            self._triangle_pressed = True
                            self._x_locked = not self._x_locked
                            self._x_locked_value = val
                        elif (
                            self._triangle_pressed and not input_vals["button_triangle"]
                        ):
                            self._triangle_pressed = False
                        if self._x_locked:
                            val = self._x_locked_value
                    elif k == "y" and self._x_locked:
                        val = eval(expr, {}, input_vals)
                        y_max = math.sqrt(1 - self._x_locked_value**2)
                        if val < -y_max:
                            val = -y_max
                        elif val > y_max:
                            val = y_max

                    else:
                        val = eval(expr, {}, input_vals)
                    setattr(vel_vec, k, scale * val)
                except NameError:
                    # some names are not defined
                    pass

        self._pub.publish(to_pub)

    def l2_r2_heightfunction(self, input_vals):
        scale = 0.01
        self._height -= input_vals["axis_l2"] * scale
        self._height += input_vals["axis_r2"] * scale
        if self._height < -1.0:
            self._height = -1.0
        if self._height > 1.0:
            self._height = 1.0
        return self._height


def main():
    rclpy.init()
    node = rclpy.create_node("ds4_twist")

    StatusToTwist(node)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
