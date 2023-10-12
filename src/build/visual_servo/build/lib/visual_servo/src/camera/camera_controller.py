import rclpy
import sys
import os
import cv2
import numpy as np
# sys.path.insert(1, full_path)
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Int8, Bool
from sensor_msgs.msg import Image
from siyi_control.siyi_sdk import SIYISDK
from time import sleep
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
### TODO This is completely unsused and should probably be deleted. The SIYI folder too
# The purpose of this, which is not implemented, is to control the gimbal and the camera

_ZR30_SERVER_IP =               "192.168.144.25"
_ZR30_SERVER_PORT =             37260

_PUBLISH_PERIOD_SEC =           0.05
_QUEUE_SIZE =                   1
_GIMBAL_KP =                    2
_GIMBAL_ERR_THRESH =            5

class CameraControllerNode(Node):
    def __init__(self, camera: SIYISDK, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__()
        self.camera = camera
        
        self.capture = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')
        self.bridge = CvBridge()


        
    def publish_image(self):
        ret, frame = self.capture.read()
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')

        
    def publish_data(self) -> None:
        """
        Will publish the current gimbal attitude and zoom level.
        """
        self.get_attitude_callback()
        self.get_zoom_callback()
        self.i += 1

    def get_attitude_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the current roll, pitch and yaw of the ZR30 Camera.
        """
        msg = Vector3Stamped()

        yaw, pitch, roll = self.camera.getAttitude()
        msg.vector.x = yaw
        msg.vector.y = pitch
        msg.vector.z = roll
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = _GIMBAL_FRAME_ID

        self.att_publisher_.publish(msg)
        #print("Publicando posicion")


    def set_attitude_callback(self, msg: Vector3Stamped) -> None:
        """
        Will listen for Vector3Stamped messages to toggle the attitude of the gimbal.

        Args:
            msg (Vector3Stamped): The message containing the attitude to set in the following format
            msg.vector.x = yaw 
            msg.vector.y = pitch
            msg.vector.z = roll (will be ignored)
        """
        
        pitch = msg.vector.y
        yaw = msg.vector.x
        self.get_logger().info(f"Gimbal attitude set to ({pitch}, {yaw}) (pitch, yaw).")
        self.camera.setGimbalRotation(yaw, pitch, err_thresh=_GIMBAL_ERR_THRESH, kp=_GIMBAL_KP)
        position_reached_msg = Bool()
        position_reached_msg.data = True
        self.position_reached_publisher.publish(position_reached_msg)
        self.camera.requestFollowMode()
        
    def set_speed_callback(self, msg: Vector3Stamped) -> None:
        """
        Will listen for Vector3Stamped messages to toggle the attitude of the gimbal.

        Args:
            msg (Vector3Stamped): The message containing the attitude to set in the following format
            msg.vector.x = yaw speed
            msg.vector.y = pitch speed
            msg.vector.z = roll (will be ignored)
        """
        
        pitch_speed = int(msg.vector.y)
        yaw_speed = int(msg.vector.x)
        self.camera.requestGimbalSpeed(yaw_speed, pitch_speed)

        
    

    def get_zoom_callback(self) -> None:
        """
        Will use the siyi_sdk to publish the zoom level of the ZR30 Camera.
        """
        msg = Float32()
        zoom_level = float(self.camera.getZoomLevel())
        msg.data = zoom_level

        self.zoom_publisher_.publish(msg)
        


    def set_zoom_callback(self, msg: Float32) -> None:
        """
        Will listen for Vector3Stamped messages to toggle the attitude of the gimbal.

        Args:
            msg (Float32): The message containing the zoom level to set.
            msg.data = zoom level
        """
        val = msg.data

        if val == 1.0 or val == 30.0:
            self._request_zoom(val)
        
        self.camera.setZoomLevel(val)
        self.get_logger().info(f"Zoom level set to {val}.")

    def set_focus_callback(self, msg: Int8) -> None:
        """
        Will listen for Int8 messages to toggle the focus of the camera.

        Args:
            msg (Int8): The message containing the focus level to set.
            msg.data = -1 (close shot), 0 (stop focus), 1 (far shot), 2 (auto)
        """
        val = msg.data
        
        if (val == 2):
            self.camera.requestAutoFocus()
            self.get_logger().info(f"Auto focus requested.")
        elif(val == 1):
            self.camera.requestLongFocus()
            self.get_logger().info(f"Long focus requested.")
        elif(val == 0):
            self.camera.requestFocusHold()
            self.get_logger().info(f"Focus hold requested.")
        elif(val == -1):
            self.camera.requestCloseFocus()
            self.get_logger().info(f"Close focus requested.")
        else:
            self.get_logger().error(f"Invalid focus argument {val}.")
            return
        
    def _request_zoom(self, zoom) -> None:
        """
        Will zoom in or out on the camera and return the zoom level.
        Zoom level: min = 1.0, max = 30.0
        Args:
            zoom: whether to zoom in (zoom = 1) or zoom out (zoom = 0)
        """
        cam_zoom = float(self.camera.getZoomLevel())
        print("Initial zoom level", cam_zoom)

        if zoom < cam_zoom:
            while zoom < cam_zoom:
                self.camera.requestZoomOut()
                cam_zoom = float(self.camera.getZoomLevel())
        elif zoom > cam_zoom:
            while zoom > cam_zoom:
                self.camera.requestZoomIn()
                cam_zoom = float(self.camera.getZoomLevel())
        else:
            pass

        # if zoom == 1:
        #     print("Zooming in")        
        #     val = self.camera.requestZoomIn()
        #     sleep(1)
        # elif zoom == -1:
        #     print("Zooming out")
        #     val = self.camera.requestZoomOut()
        #     sleep(1)
        # else:
        #     print("Wrong input to zoom. Input 1 or -1.")
        #     pass

        val = self.camera.requestZoomHold()
        sleep(1)
        cam_zoom = float(self.camera.getZoomLevel())
        sleep(1)

        print("Achieved zoom level: ", cam_zoom)


def main(args=None):
    camera = SIYISDK(server_ip=_ZR30_SERVER_IP, port=_ZR30_SERVER_PORT)
    camera.connect()

    rclpy.init(args=args)
    node = CameraControllerNode(camera=camera)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    camera.disconnect()

if __name__ == "__main__":
    main()
