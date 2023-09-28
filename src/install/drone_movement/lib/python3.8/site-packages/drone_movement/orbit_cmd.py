"""
Python implementation of Orbit command

"""


import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')

        # Initialise publisher nodes
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Set timer
        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.offboard_setpoint_counter_ = 0

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()


        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        # self.publish_trajectory_setpoint()

        # Publish orbit command
        self.publish_orbit_command()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def arm(self):
        '''
        Arm the vehicle
        '''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        '''
        Disarm the vehicle
        '''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def publish_offboard_control_mode(self):
        '''
        Publish the offboard control mode.
        For this example, only position and altitude controls are active.
        '''
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        '''
        Publish a trajectory setpoint
        For this example, it sends a trajectory setpoint to make the
        vehicle hover at 5 meters with a yaw angle of 180 degrees.
        '''
        msg = TrajectorySetpoint()
        # msg.timestamp = self.timestamp_
        msg.position = [5.0, 0.0, -5.0] 
        # msg.velocity = [5.0, 0.0, -5.0] 
        msg.yaw = -3.14  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_orbit_command(self):
        '''
        Start orbiting on the circumference of a circle defined by the parameters. 		
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_ORBIT  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        
        # Orbit parameters
        # |Radius [m] |Velocity [m/s] |Yaw behaviour |Empty |Latitude/X |Longitude/Y |Altitude/Z |
        # uint16 VEHICLE_CMD_DO_ORBIT = 34	
        msg.param1 = 3.0
        msg.param2 = 1.0
        msg.param3 = 0.0
        # msg.param4 = orbits
        msg.x = 5      # latitude
        msg.y = 5      # longtitude
        msg.z = 5.0      # altitude

        self.vehicle_command_publisher_.publish(msg)

        self.get_logger().info("Orbit command sent")
        

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
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...py\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
