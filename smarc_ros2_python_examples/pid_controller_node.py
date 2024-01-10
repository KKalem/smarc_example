#!/usr/bin/python3

import rclpy
import sys

from pid_model import PIDModel


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('pid_controller_node')