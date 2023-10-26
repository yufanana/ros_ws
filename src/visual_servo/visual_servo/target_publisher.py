#!/usr/bin/env python

from __future__ import annotations

import rclpy
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
# from vservo_interfaces.msg import TargetOffset

_TARGET_SUB_TOPIC = "visual_servo/target_offset"


class LocalControlHandler(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.initPubs()
        self.initSubs()

        freq = 10
        self.iterateTimer = self.create_timer(1/freq, self.iterate)
        self.count = 0

    def initPubs(self):
        self.target_pub = self.create_publisher(Vector3,
                                                _TARGET_SUB_TOPIC, self.qos_profile)

    def initSubs(self):
        # keyboard input?
        pass

    def iterate(self):
        self.count += 1
        # relative position of target between [-1,1], with 0 as the frame centre
        # +ve --> right half of frame, -ve --> left half of frame

        msg = Vector3()
        msg.x = 0.5*np.cos(self.count*0.05)
        msg.y = 0.5*np.cos(self.count*0.05)
        msg.z = 0.3

        print(f"Publishing target offset --> {msg.x}")
        self.target_pub.publish(msg)


def main(args=None):
    print("Starting offboard control node...")

    rclpy.init(args=args)

    offboard_control = LocalControlHandler()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
