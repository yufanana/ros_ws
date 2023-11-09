#!/usr/bin/env python

from __future__ import annotations

import rclpy
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
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

        self.freq = 10
        self.iterateTimer = self.create_timer(1/self.freq, self.iterate)
        self.count = 0
        self.hold_count = 0
        self.hold = True
        self.reverse = 1
        self.iterate_count = True

    def initPubs(self):
        self.target_pub = self.create_publisher(Vector3Stamped,
                                                _TARGET_SUB_TOPIC, self.qos_profile)

    def initSubs(self):
        pass

    def iterate(self):
        # relative position of target between [-1,1], with 0 as the frame centre
        # +ve --> right half of frame, -ve --> left half of frame


        # period = 20.0    # arbitrary unit
        # x = 0.25*np.sin(self.count/period)
        # if abs(x) > 0.2:
        #     x = np.sign(x)*0.2
        # self.count += 1
        # x += 0.2

        # Simple cosine
        msg = Vector3Stamped()
        self.count += 1
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.vector.x = 0.5*np.cos(self.count*0.05)
        msg.vector.y = 0.5*np.cos(self.count*0.05)
        msg.vector.z = 0.1+0.001*self.count      # bbsize, proportion of frame

        print(f"Publishing --> x: {msg.vector.x}, bb_size: {msg.vector.z}")
        self.target_pub.publish(msg)


def main(args=None):
    print("Starting offboard control node...")

    rclpy.init(args=args)

    offboard_control = LocalControlHandler()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
