from geometry_msgs.msg import PointStamped


class position_publishers():
    '''
    This is only for logging purposes
    '''

    def __init__(self, parent):
        self.parent = parent

        self.create_publishers()

        self.parent.create_timer(0.01,
                                 self.publish_all_odometry)

        self.camera_msg = PointStamped()
        self.camera_raw_msg = PointStamped()
        self.px4_msg = PointStamped()
        self.optitrack_msg = PointStamped()

    def create_publishers(self):
        self.camera_pub = self.parent.create_publisher(PointStamped,
                                                       'camera_odometry',
                                                       self.parent.qos_profile)
        self.camera_raw_pub = self.parent.create_publisher(PointStamped,
                                                           'camera_raw_odometry',
                                                           self.parent.qos_profile)
        self.px4_pub = self.parent.create_publisher(PointStamped,
                                                    'px4_odometry',
                                                    self.parent.qos_profile)
        self.optitrack_pub = self.parent.create_publisher(PointStamped,
                                                          'optitrack_odometry',
                                                          self.parent.qos_profile)

    def publish_all_odometry(self):
        self.camera_msg = self.publish_odometry(self.camera_msg, self.camera_pub,
                                                self.parent.camera.x, self.parent.camera.y, self.parent.camera.z)

        self.camera_raw_msg = self.publish_odometry(self.camera_raw_msg, self.camera_raw_pub,
                                                    self.parent.camera.x_raw, self.parent.camera.y_raw, self.parent.camera.z_raw)

        if self.parent.odometry.is_odometry_recieved():
            self.px4_msg = self.publish_odometry(self.px4_msg, self.px4_pub,
                                                 -self.parent.odometry.y, -self.parent.odometry.x, self.parent.odometry.z)

        self.optitrack_msg = self.publish_odometry(self.optitrack_msg, self.optitrack_pub,
                                                   self.parent.optitrack.x, self.parent.optitrack.y, self.parent.optitrack.z)

    def publish_odometry(self, point_stamped: PointStamped, publisher, x, y, z):
        # Don't publish if there is nothing to publish
        if x is None or y is None or z is None:
            return point_stamped
        # Don't publish if there is nothing new to publish
        if point_stamped.point.x == x and point_stamped.point.y == y and point_stamped.point.z == z:
            return point_stamped

        point_stamped.point.x = float(x)
        point_stamped.point.y = float(y)
        point_stamped.point.z = float(z)

        publisher.publish(point_stamped)

        return point_stamped
