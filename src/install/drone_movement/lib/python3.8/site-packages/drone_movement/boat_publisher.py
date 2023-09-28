#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy
from geometry_msgs.msg import Point

# from pytimedinput import timedInput


class BoatPublisher(Node):

    def __init__(self):
        super().__init__('boat_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.publisher_boat_position = self.create_publisher(Point,
                                                             "boat/position",
                                                             qos_profile)

        # self.boat_queue = []
        
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        # boat_position, timed_out = timedInput("Enter boat XY-position: ", timeout=1)
        # if not timed_out:
        #     boat_position = [float(i) for i in boat_position.split(" ")]
        #     boat_position.append(0.)
        #     self.boat_queue.append(boat_position)

        boat_position = input("Enter boat XY-position: ")
        boat_position = [float(i) for i in boat_position.split(" ")]
        
        point_msg = Point()
        point_msg.x = boat_position[0]
        point_msg.y = boat_position[1]
        point_msg.z = 0.0
        
        self.publisher_boat_position.publish(point_msg)
        print(f"Published position: x={point_msg.x} y={point_msg.y}")

def main(args=None):
    rclpy.init(args=args)

    print("---------------------------------")
    offboard_control = BoatPublisher()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()