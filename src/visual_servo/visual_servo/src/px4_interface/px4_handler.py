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

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..offboard_control import OffboardControl


from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
# from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitudeSetpoint

import numpy as np


class PX4Handler():
    def __init__(self, parent) -> None:
        self.parent: OffboardControl  # set type hinting for parentObj

        self.parent = parent

        self.qos_profile = self.parent.qos_profile

        self.initPubs()
        # self.initSubs()

        self.nav_state = None

    def initPubs(self):
        self.publisher_offboard_mode = self.parent.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos_profile)
        self.publisher_trajectory = self.parent.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
        self.publisher_command = self.parent.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        self.publisher_attitude = self.parent.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', self.qos_profile)

    # def initSubs(self):
    #     # This is currently broken
    #     # self.status_sub = self.parent.create_subscription(
    #     #     VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.qos_profile)
    #     pass

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def isArmed(self):
        if self.nav_state is None:
            return False
        return self.nav_state == VehicleStatus.STATE_ARMED

    def isLanded(self):
        return self.px4Handler.getLandedState() == 1

    def arm(self):
        command_msg_arm = VehicleCommand()
        command_msg_arm.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_arm.param1 = 1.0
        command_msg_arm.param2 = 0.0
        command_msg_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command_msg_arm.target_component = 1
        command_msg_arm.target_system = 1
        command_msg_arm.source_component = 1
        command_msg_arm.source_system = 1
        command_msg_arm.from_external = True
        # if self.cnt >= 2/self.dt:
        self.publisher_command.publish(command_msg_arm)

    def disarm(self):
        command_msg_arm = VehicleCommand()
        command_msg_arm.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_arm.param1 = 0.0
        command_msg_arm.param2 = 0.0
        command_msg_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command_msg_arm.target_component = 1
        command_msg_arm.target_system = 1
        command_msg_arm.source_component = 1
        command_msg_arm.source_system = 1
        command_msg_arm.from_external = True

        self.publisher_command.publish(command_msg_arm)

    def takeoff(self):
        command_msg_takeoff = VehicleCommand()
        command_msg_takeoff.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_takeoff.param1 = 1.0
        command_msg_takeoff.param2 = 4.0
        command_msg_takeoff.param3 = 2.0
        command_msg_takeoff.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command_msg_takeoff.target_component = 1
        command_msg_takeoff.target_system = 1
        command_msg_takeoff.source_component = 1
        command_msg_takeoff.source_system = 1
        command_msg_takeoff.from_external = True
        self.publisher_command.publish(command_msg_takeoff)

    def land(self):
        command_msg_land = VehicleCommand()
        command_msg_land.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_land.param1 = 1.0
        command_msg_land.param2 = 4.0
        command_msg_land.param3 = 6.0
        command_msg_land.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command_msg_land.target_component = 1
        command_msg_land.target_system = 1
        command_msg_land.source_component = 1
        command_msg_land.source_system = 1
        command_msg_land.from_external = True
        self.publisher_command.publish(command_msg_land)

    def offboard(self):
        # Set to offboard control mode in the topic '/fmu/in/vehicle_command'
        command_msg_mode = VehicleCommand()
        command_msg_mode.timestamp = int(Clock().now().nanoseconds / 1000)
        command_msg_mode.param1 = 1.0
        command_msg_mode.param2 = 6.0
        command_msg_mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command_msg_mode.target_component = 1
        command_msg_mode.target_system = 1
        command_msg_mode.source_component = 1
        command_msg_mode.source_system = 1
        command_msg_mode.from_external = True
        self.publisher_command.publish(command_msg_mode)

    def attitude_mode(self):
        # Publish attitude setpoint offboard control mode in the topic '/fmu/in/offboard_control_mode'
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True
        offboard_msg.body_rate = False
        offboard_msg.actuator = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def setAttitudeReference(self, roll, pitch, yaw, thrust, yaw_rate=1):
        def euler_to_quaternion(yaw, pitch, roll):
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
                np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
                np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
                np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
                np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            return [qw, qx, qy, qz]

        attitude_msg = VehicleAttitudeSetpoint()
        # print("traj setpoint.position:{:.2f}, {:.2f}, {:.2f}".format(x, y, z))

        attitude_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        attitude_msg.q_d = euler_to_quaternion(yaw, pitch, roll)

        # attitude_msg.yaw_sp_move_rate = float(yaw_rate)

        # For clarification: For multicopters thrust_body[0] and thrust[1] are
        # usually 0 and thrust[2] is the negative throttle demand.
        # Normalized thrust command in body NED frame [-1,1] float32[3]
        attitude_msg.thrust_body = [0.0, 0.0, thrust]

        self.publisher_attitude.publish(attitude_msg)

    def setPositionReference(self, target_position):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = target_position[0]
        trajectory_msg.position[1] = target_position[1]
        trajectory_msg.position[2] = target_position[2]
        trajectory_msg.timestamp = int(
            Clock().now().nanoseconds / 1000)  # microseconds
        self.publisher_trajectory.publish(trajectory_msg)
