import numpy as np

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..offboard_control import OffboardControl

from px4_msgs.msg import VehicleOdometry


class px4_odometry():
    def __init__(self, parent):
        self.parent: OffboardControl
        self.parent = parent

        # time since system start (microseconds) uint64
        self.timestamp = None
        self.timestamp_sample = None

        # uint8 POSE_FRAME_UNKNOWN = 0
        # uint8 POSE_FRAME_NED     = 1 # NED earth-fixed frame
        # uint8 POSE_FRAME_FRD     = 2 # FRD world-fixed frame, arbitrary heading reference
        # Position and orientation frame of reference uint8
        self.pose_frame = None

        # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown float32[3]
        self.position = None
        self.x = None
        self.y = None
        self.z = None

        # Quaternion rotation from FRD body frame to reference frame. First value NaN if invalid/unknown float32[4]
        self.q = None

        # Attitude in euler angles measures in radians
        self.attitude = None
        self.rx = None
        self.ry = None
        self.rz = None

        # uint8 VELOCITY_FRAME_UNKNOWN  = 0
        # uint8 VELOCITY_FRAME_NED      = 1 # NED earth-fixed frame
        # uint8 VELOCITY_FRAME_FRD      = 2 # FRD world-fixed frame, arbitrary heading reference
        # uint8 VELOCITY_FRAME_BODY_FRD = 3 # FRD body-fixed frame
        # Reference frame of the velocity data uint8
        self.velocity_frame = None

        # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown float32[3]
        self.velocity = None
        self.v_x = None
        self.v_y = None
        self.v_z = None

        # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown float32[3]
        self.angular_velocity = None

        # float32[3]
        self.position_variance = None
        self.orientation_variance = None
        self.velocity_variance = None

        # uint8
        self.reset_counter = None
        # int8
        self.quality = None

        # TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
        # TOPICS estimator_odometry

        self.create_subscriber()

    def create_subscriber(self):
        self.odometry_subscriber = self.parent.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.update_odometry,
            self.parent.qos_profile
        )

        self.odometry_subscriber

    def is_odometry_recieved(self):
        if self.position is not None:
            return True
        else:
            return False

    def update_odometry(self, msg):
        self.timestamp = msg.timestamp
        self.timestamp_sample = msg.timestamp_sample
        self.pose_frame = msg.pose_frame
        self.position = msg.position
        self.q = msg.q
        self.velocity_frame = msg.velocity_frame
        self.velocity = msg.velocity
        self.angular_velocity = msg.angular_velocity
        self.position_variance = msg.position_variance
        self.orientation_variance = msg.orientation_variance
        self.velocity_variance = msg.velocity_variance
        self.reset_counter = msg.reset_counter
        self.quality = msg.quality

        self.attitude = euler_from_quaternion(self.q)

        self.x = self.position[0]
        self.y = self.position[1]
        self.z = self.position[2]

        self.v_x = self.velocity[0]
        self.v_y = self.velocity[1]
        self.v_z = self.velocity[2]

        self.rx = self.attitude[0]
        self.ry = self.attitude[1]
        self.rz = self.attitude[2]


# From https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]
    w = quaternion[0]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
