#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy
from geometry_msgs.msg import Point


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
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        boat_position = input("Enter boat XY-position: ")
        boat_position = [float(i) for i in boat_position.split(" ")] # receive in ENU

        # Convert to NED for PX4 by swapping x and y
        point_msg = Point()
        point_msg.x = boat_position[1]
        point_msg.y = boat_position[0]
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
    