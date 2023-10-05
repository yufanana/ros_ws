#!/usr/bin/env python

from __future__ import annotations

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import Float32

import numpy as np

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
        self.target_pub = self.create_publisher(
            Float32, '/targetFrameOffset', self.qos_profile)

    def initSubs(self):
        # keyboard input?
        pass

    def iterate(self):
        self.count += 1
        msg = Float32()
        # relative position of target between [-1,1], with 0 as the frame centre
        # +ve --> right half of frame, -ve --> left half of frame
        
        msg.data = 0.2 - self.count*0.0005
        
        if msg.data <= 0.0:
            msg.data = 0.0

        print(f"Publishing frame offset --> {msg.data}")
        self.target_pub.publish(msg)


def main(args=None):
    print("Starting offboard control node...")

    rclpy.init(args=args)

    offboard_control = LocalControlHandler()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
