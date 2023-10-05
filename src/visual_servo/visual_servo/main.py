import rclpy

from .src.offboard_control import OffboardControl


def main(args=None):
    print("Starting offboard control node...")
    # input("Press Enter to continue...")
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
