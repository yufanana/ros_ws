#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

from __future__ import annotations
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node

import numpy as np

__author__ = "Marios Nektarios Stamatopoulos"
__contact__ = "marsta@ltu.se"


def cart2pol(x, y):
    """Convert cartesian coordinates to polar coordinates

    Args:
        x (float): x-coordinate
        y (float): y-coordinate
    """
    r = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)  # -pi to pi

    r, phi = float(r), float(phi)

    return r, phi


class LocalControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.initPubs()
        self.initSubs()

        self.path = []
        self.pathIndex = 0
        self.path.append([2.0, 2.0, 2.0])
        # self.path.append([7, 4, 2])

        freq = 10
        self.iterateTimer = self.create_timer(1/freq, self.iterate)

    def initPubs(self):
        self.polarRefPub = self.create_publisher(
            PoseStamped, '/polarRef', self.qos_profile)

    def initSubs(self):
        self.positionSub = self.create_subscription(
            PoseStamped, '/poseReference', self.poseReferenceCallback,
            self.qos_profile)

    def poseReferenceCallback(self, msg):
        pass

    def iterate(self):
        r, phi = cart2pol(self.path[self.pathIndex][0], 
                          self.path[self.pathIndex][1])
        msg = PoseStamped()
        msg.pose.position.x = r
        msg.pose.position.y = phi
        msg.pose.position.z = self.path[self.pathIndex][2]

        print("Publishing polar reference --> r:{:.2f} phi:{:.2f} z:{:.2f}".format(
            r, phi, self.path[self.pathIndex][2]))
        self.polarRefPub.publish(msg)


def main(args=None):
    print("Starting offboard control node...")
    # input("Press Enter to continue...")
    rclpy.init(args=args)

    offboard_control = LocalControlHandler()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
