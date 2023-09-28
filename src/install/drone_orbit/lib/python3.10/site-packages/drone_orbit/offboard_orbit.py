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
from geometry_msgs.msg import Point

# from tf_transformations import euler_from_quaternion

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.subscriber_boat_position = self.create_subscription(Point,
                                                     "/boat/position",
                                                     self.boat_position_callback,
                                                     qos_profile_sensor_data)
        self.subscriber_status = self.create_subscription(VehicleStatus,
                                                          '/fmu/out/vehicle_status',
                                                          self.vehicle_status_callback,
                                                          qos_profile)
        self.subscriber_odom = self.create_subscription(VehicleOdometry,
                                                     "/fmu/out/vehicle_odometry",
                                                     self.odom_callback,
                                                     qos_profile_sensor_data)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 
                                                             '/fmu/in/offboard_control_mode', 
                                                             qos_profile)
        self.publisher_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, 
                                                          '/fmu/in/trajectory_setpoint', 
                                                          qos_profile)
        self.publisher_vehicle_cmd = self.create_publisher(VehicleCommand, 
                                                               "/fmu/in/vehicle_command", 
                                                               qos_profile)

        # Callback function for the arm timer
        arm_timer_period = 0.1      # should be > 2 Hz
        self.arm_counter = 0
        self.arm_timer = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # Callback function for the command timer
        self.cmd_timer_period = 0.02  # seconds
        self.cmd_timer = self.create_timer(self.cmd_timer_period, self.cmd_timer_callback)
        self.dt = self.cmd_timer_period

        # Utility variables for mission commands
        self.theta = 0.0
        self.boat_queue = []
        self.cmd_state = 'TAKEOFF'
        self.target_position = []       # NED frame
        self.path = []
        self.path_counter = 0
        self.timeout = 5.0        # seconds
        self.timeout_timer = 0

        # Drone status
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.drone_odom = VehicleOdometry()

        # CONSTANTS
        self.TAKEOFF_HEIGHT = -3.0
        self.HOME_HEIGHT = -8.0
        self.ORBIT_HEIGHT = -5.0
        self.ORBIT_RADIUS = 5.0
        self.ORBIT_OMEGA = 0.2


    def boat_position_callback(self, msg):
        '''
        Subscriber callback function to add new boat positions to the boat queue.
        '''
        self.boat_queue.append([msg.x,msg.y])
        self.get_logger().info(f'Updated boat_queue: {self.boat_queue}')


    def vehicle_status_callback(self, msg):
        '''
        Subscriber callback function to update the vehicle status of the drone.
        '''
        self.nav_state = msg.nav_state
        # print("Current NAV_STATUS: ", msg.nav_state)
        # print("Desired offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)


    def odom_callback(self, msg):
        '''
        Subscriber callback function to update the odometry of the drone.
        '''
        self.drone_odom = msg


    def takeoff(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0,0.0,self.TAKEOFF_HEIGHT]
        trajectory_msg.yaw = np.pi/2
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
        self.publisher_trajectory_setpoint.publish(trajectory_msg)

    def setpoint_reached(self, target_position):
        '''
        Returns true when the drone is near the target position
            target_position     [x,y,z] in in NED frame
        '''
        margin = 0.4
        if np.linalg.norm(self.drone_odom.position-target_position) < margin:
            # self.get_logger().info(f"Setpoint {target_position} reached")
            return True
        return False


    def generate_path(self,target_position, abs_speed=0.5):
        '''
        Obtain an array of setpoints along a straight line using linear interpolation 
        with constant velocity based on a specified maximum speed. 
            target_position     [x,y,z] in in NED frame
            abs_speed           absolute speed in m/s
        '''
        current_position = self.drone_odom.position
        delta = np.linalg.norm(target_position-current_position)/abs_speed  # time step
        steps = int(delta/self.cmd_timer_period)
        # yaw = np.arctan2(target_position[1],target_position[0])
        vx = (target_position[0]-current_position[0])/delta
        vy = (target_position[1]-current_position[1])/delta
        vz = (target_position[2]-current_position[2])/delta
        yaw = np.arctan2(vy,vx)
        for i in range(steps+1):
            point = TrajectorySetpoint()
            point.position[0] = current_position[0]+(target_position[0]-current_position[0])/steps*i
            point.position[1] = current_position[1]+(target_position[1]-current_position[1])/steps*i
            point.position[2] = current_position[2]+(target_position[2]-current_position[2])/steps*i
            point.velocity = [vx,vy,vz]
            point.yaw = yaw
            self.path.append(point)

        # for i in range(steps+1):
 
    def publish_vehicle_cmd(self, command, param1=0.0, param2=0.0):
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
        msg.target_component = 1    # component which should execute the command
        msg.source_system = 1       # system sending the command
        msg.source_component = 1    # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_vehicle_cmd.publish(msg)


    def publish_orbit_cmd(self,target_position):
        '''
        Publish a stream trajectory setpoints messages to travel clockwise in a circle
        with the drone facing centre of orbit.
        '''
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = target_position[0]+self.ORBIT_RADIUS*np.cos(self.theta)
        trajectory_msg.position[1] = target_position[1]+self.ORBIT_RADIUS*np.sin(self.theta)
        trajectory_msg.position[2] = target_position[2]
        # trajectory_msg.velocity[0] = self.ORBIT_RADIUS*self.ORBIT_OMEGA*np.cos(self.theta) # not sure
        # trajectory_msg.velocity[1] = self.ORBIT_RADIUS*self.ORBIT_OMEGA*np.sin(self.theta) # not sure
        # trajectory_msg.velocity[2] = 0.0
        trajectory_msg.yaw = self.theta+np.pi
        trajectory_msg.yawspeed = self.ORBIT_OMEGA
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
        self.publisher_trajectory_setpoint.publish(trajectory_msg)

        self.theta = self.theta + self.ORBIT_OMEGA*self.dt


    def arm_timer_callback(self):
        '''
        Callback function to arm the drone in offboard mode.
        '''
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.attitude = True
        offboard_msg.acceleration = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if (self.arm_counter == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Set to offboard control mode sent")
            # Arm the vehicle
            self.publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Arm command sent")

        # Stop the counter after reaching 20
        if (self.arm_counter < 21):
            self.arm_counter += 1

    def cmd_timer_callback(self):
        '''
        Callback function to command different stages of the drone mission
        '''
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # print("cmd_state: ",self.cmd_state)

            if self.cmd_state == 'TAKEOFF':
                self.takeoff()
                # Check if reached, then transit to next state
                if self.setpoint_reached([0.0,0.0,self.TAKEOFF_HEIGHT]):
                    self.get_logger().info("Reached TAKEOFF_HEIGHT")
                    self.cmd_state = 'SCOUT'
                    self.get_logger().info("Moving to home position")


            if self.cmd_state == 'SCOUT':
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position = [0.0,0.0,self.HOME_HEIGHT]
                trajectory_msg.yaw = np.pi/2
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
                self.publisher_trajectory_setpoint.publish(trajectory_msg)
                # Check if reached, then transit to next state
                if self.setpoint_reached([0.0,0.0,self.HOME_HEIGHT]) and len(self.boat_queue)!=0:
                    self.cmd_state = 'HOME'


            if self.cmd_state == 'READY':
                # Check for boat in boat queue
                if len(self.boat_queue) != 0:     # not empty
                    target_x,target_y = self.boat_queue[0]
                    self.target_position = [target_x,target_y,self.ORBIT_HEIGHT]
                    self.generate_path([target_x+self.ORBIT_RADIUS,target_y,self.ORBIT_HEIGHT],abs_speed=1.0)
                    self.cmd_state = 'MOVE'
                    self.get_logger().info(f"Moving to {[target_x+self.ORBIT_RADIUS,target_y,self.ORBIT_HEIGHT]}")
                else:
                    self.get_logger().info("Empty boat queue. Going home.")
                    self.generate_path([0.0,0.0,self.HOME_HEIGHT],abs_speed=1.0)
                    self.cmd_state = 'MOVE'


            if self.cmd_state == 'HOME':
                # Check for boat in boat queue
                if len(self.boat_queue) != 0:     # not empty
                    target_x,target_y = self.boat_queue[0]
                    self.target_position = [target_x,target_y,self.ORBIT_HEIGHT]
                    self.generate_path([target_x+self.ORBIT_RADIUS,target_y,self.ORBIT_HEIGHT],abs_speed=1.0)
                    self.cmd_state = 'MOVE'
                    self.get_logger().info(f"Moving to {[target_x+self.ORBIT_RADIUS,target_y,self.ORBIT_HEIGHT]}")
                else:
                    # Hover until timeout, then land
                    self.timeout_timer += 1
                    # Reorientate to face +y / forward
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position = [0.0,0.0,self.HOME_HEIGHT]
                    trajectory_msg.yaw = np.pi/2
                    trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)
                    if self.timeout_timer >= self.timeout/self.cmd_timer_period:
                        self.publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.cmd_state = 'LAND'
                        self.get_logger().info("Timeout. Landing...")

            
            if self.cmd_state == 'MOVE':
                trajectory_msg = self.path[self.path_counter]
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000) # microseconds
                self.publisher_trajectory_setpoint.publish(trajectory_msg)
                self.path_counter += 1
                # print("d: ", np.linalg.norm(self.drone_odom.position-self.path[-1].position))
                # Check if reached, then transit to next state
                if self.setpoint_reached(self.path[-1].position):
                    self.get_logger().info(f"Reached {self.path[-1].position}")
                    # Reset path variables
                    self.path_counter = 0
                    self.path = []
                    if self.target_position is not None:
                        # Has a target position to orbit around
                        self.get_logger().info(f"Orbitting around {self.target_position}")
                        self.cmd_state = 'ORBIT'
                    else:
                        # No target positions to orbit around
                        self.cmd_state = 'HOME'


            if self.cmd_state == 'ORBIT':
                self.publish_orbit_cmd(self.target_position)
                # End of orbit, reset for next boat
                if self.theta >= 2*np.pi:
                    self.get_logger().info("Orbit completed")
                    self.boat_queue.pop(0)
                    self.theta = 0.0
                    self.target_position = None
                    self.cmd_state = 'READY'
                    

            if self.cmd_state == 'LAND':
                # self.destroy_node()
                # rclpy.shutdown()
                pass


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    # offboard_control.boat_queue = [[2.0,2.0]]   # for ease of testing
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

