#!/usr/bin/env python

from __future__ import annotations

import rclpy
import numpy as np
from time import sleep
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from visual_servo_interfaces.msg import TuneController

_TARGET_PUB_TOPIC = "tune_controller"

class LocalControlHandler(Node):
    def __init__(self):
        super().__init__('tune_controller')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(TuneController,
                                                _TARGET_PUB_TOPIC, self.qos_profile)


def main(args=None):

    rclpy.init(args=args)
    sent = False
    offboard_control = LocalControlHandler()

    while rclpy.ok():
        if not sent:
            msg = TuneController()
            msg.ss_roll = -0.01
            msg.ss_pitch = -0.0183
            msg.ss_thrust = -0.35
            msg.height_kp = 3.0
            msg.ref_height = 3.0
            offboard_control.publisher.publish(msg)
            offboard_control.publisher.publish(msg)

            print(f'Published TuneController msg: \n \t ss_roll: {msg.ss_roll} \
                  \n \t ss_pitch: {msg.ss_pitch} \n \t ss_thrust: {msg.ss_thrust} \
                  \n \t height_kp: {msg.height_kp} \n \t ref_height: {msg.ref_height} ')
            # print("Published TuneController msg")
            sent = True
        else:
            break
        
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
