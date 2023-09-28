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

__author__ = ""
__contact__ = ""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy, qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point

from .px4_transforms import *

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.subscriber_odom = self.create_subscription(VehicleOdometry,
                                                     "/fmu/out/vehicle_odometry",
                                                     self.odom_callback,
                                                     qos_profile_sensor_data)
        self.publisher_odom = self.create_publisher(PoseWithCovarianceStamped, 
                                               'drone_posecov',
                                               10)
        self.subscriber_boat_position = self.create_subscription(Point,
                                                     "/boat/position",
                                                     self.boat_position_callback,
                                                     qos_profile_sensor_data)
        self.subscriber_status = self.create_subscription(VehicleStatus,
                                                          '/fmu/out/vehicle_status',
                                                          self.vehicle_status_callback,
                                                          qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 
                                                             '/fmu/in/offboard_control_mode', 
                                                             qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 
                                                          '/fmu/in/trajectory_setpoint', 
                                                          qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, 
                                                               "/fmu/in/vehicle_command", 
                                                               qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pwcs = PoseWithCovarianceStamped()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.
        self.drone_odom = VehicleOdometry()
        self.boat_queue = []

        self.offboard_setpoint_counter_ = 0
        self.home_counter = 0
        self.begin_orbit = False

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


    def boat_position_callback(self, msg):
        self.boat_queue.append([msg.x,msg.y])
        print(self.boat_queue)
        

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state
        if msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("NAV_STATUS: ", msg.nav_state)
            print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)


    def odom_callback(self, msg):
        self.drone_odom = msg

        # self.pwcs.header.stamp = self.get_clock().now().to_msg()
        # self.pwcs.header.frame_id = "drone"
        
        # position_enu = transform_position(msg.position, "NED_2_ENU")
        # self.pwcs.pose.pose.position.x = float(position_enu[0])
        # self.pwcs.pose.pose.position.y = float(position_enu[1])
        # self.pwcs.pose.pose.position.z = float(position_enu[2])
        
        # q_enu_flu = px4_to_ros_orientation(msg.q)
        # self.pwcs.pose.pose.orientation.w = float(q_enu_flu[0])
        # self.pwcs.pose.pose.orientation.x = float(q_enu_flu[1]) 
        # self.pwcs.pose.pose.orientation.y = float(q_enu_flu[2]) 
        # self.pwcs.pose.pose.orientation.z = float(q_enu_flu[3])
        
        # pose_variance_ned = np.hstack((msg.position_variance, msg.orientation_variance))
        # pose_covariance_ned = np.eye(6) * pose_variance_ned
        # pose_covariance_enu = transform_cov6d(pose_covariance_ned, "NED_2_ENU")
        # self.pwcs.pose.covariance = pose_covariance_enu.flatten()
        
        # self.publisher_odom.publish(self.pwcs)

        
    def return_to_home(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.,0.,-5.]
        self.publisher_trajectory.publish(trajectory_msg)
        self.get_logger().info("Return to home command sent")

 
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        '''
        Publish vehicle commands
            command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
            param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
            param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
        '''
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command       # command ID
        msg.target_system = 1       # system which should execute the command
        msg.target_component = 1    # component which should execute the command, 0 for all components
        msg.source_system = 1       # system sending the command
        msg.source_component = 1    # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_vehicle_command.publish(msg)


    def publish_trajectory_setpoint(self, target_x, target_y, target_z):
        '''
        Publish a trajectory setpoint to specified (x,y,z) and hover
        '''
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [target_x,target_y,target_z]
        trajectory_msg.yaw = 0.
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
        self.publisher_trajectory.publish(trajectory_msg)


    def publish_orbit_cmd(self,target_x,target_y,radius=10.,omega=0.2):
        
        trajectory_msg = TrajectorySetpoint()
        self.get_logger().info("Performing orbit")
        if self.theta < 2*np.pi:
            # Orbit trajectory, with robot facing centre of orbit
            trajectory_msg.position[0] = target_x + radius * np.cos(self.theta)
            trajectory_msg.position[1] = target_y + radius * np.sin(self.theta)
            trajectory_msg.position[2] = -5.0
            trajectory_msg.velocity[0] = -radius * omega * np.sin(self.theta)
            trajectory_msg.velocity[1] = radius * omega * np.cos(self.theta)
            trajectory_msg.velocity[2] = 0.
            trajectory_msg.yaw = self.theta
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds

            self.theta = self.theta + omega * self.dt

        self.publisher_trajectory.publish(trajectory_msg)

    def setpoint_reached(self, target_position):
        # if reached setpoint
        margin = 0.5
        if np.linalg.norm(self.drone_odom.position-target_position) < margin:
            self.get_logger().info("Setpoint reached")
            # self.return_to_home()
            return True
        return False
    

    def do_orbit_sequence(self, target_x, target_y,radius=10.0):

        # go to start of orbit circle
        if self.begin_orbit == False:
            self.get_logger().info("Moving to start of orbit")
            self.publish_trajectory_setpoint(target_x+radius,target_y,-5.0)

            # reached start of orbit circle
            if self.setpoint_reached([target_x+radius,target_y,-5.0]):
                self.begin_orbit = True
        
        # start orbitting
        else:   # self.begin_orbit == True:
            self.publish_orbit_cmd(target_x,target_y)

            # end of orbit, return to home
            if self.theta >= 2*np.pi:
                self.theta = 0
                self.get_logger().info("Orbit completed")
                self.return_to_home()
                self.boat_queue.pop()
                self.begin_orbit = False
        

    def timer_callback(self):
        # PX4 requires that the vehicle is already receiving OffboardControlMode messages 
        # before it will arm in offboard mode, or before it will switch to offboard mode when flying.
        
        if (self.offboard_setpoint_counter_ == 20):
            # Change to Offboard mode after 20 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Arm command sent")

        # stop the counter after reaching 21
        if (self.offboard_setpoint_counter_ < 21):
            self.offboard_setpoint_counter_ += 1

        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.attitude = True
        offboard_msg.acceleration = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)


        # main offboard loop
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            if len(self.boat_queue) != 0:     # not empty
                target_x,target_y = self.boat_queue[0]
                self.do_orbit_sequence(target_x,target_y)
                

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
