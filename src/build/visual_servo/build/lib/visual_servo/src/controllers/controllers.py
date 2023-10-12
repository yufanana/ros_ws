from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..offboard_control import OffboardControl

from .pid_controllers import pid    # YF reviewed
import numpy as np
import time


# TODO
# Implement a set yaw and pitch function
#   - This should also cause the controller to be in polar mode
# Implement a set x and y function
#   - This should also cause the controller to be in xy mode
# Implement a set reference height function
#   - It would be good if the reference height would be given in a ramp, when changed, such that the drone doesn't risk loosing the marker

class position_controller():
    def __init__(self, parent, odometry_class):
        self.parent: OffboardControl = parent

        self.odometry = odometry_class

        # Height controller's negavtive values means up
        self.height_controller = pid(7.08, 0.0022, a1=-0.5, b0=7.75, b1=-7.25,
                                     gain=1, integral_limit=5, parent=self.parent)
        
        # Visual servo controller
        self.visual_servo_controller = pid(kp=0.2, gain=0.5, integral_limit=5, parent=self.parent)

        # Marker feedback controller used by camera
        # kp = 2.85 * 10**(-2)
        # gain = 1
        # ki = 1.056 * 10**(-3)
        # a1 = -0.6512
        # b0 = 16.69
        # b1 = -16.34

        # # x and y controllers in FRD frame
        # self.x_cam_controller = pid(kp, ki, a1=a1, b0=b0, b1=b1,
        #                             gain=gain, parent=self.parent)  # UAV body frame x-axis
        # self.y_cam_controller = pid(kp, ki, a1=a1, b0=b0, b1=b1,
        #                             gain=gain, parent=self.parent)  # UAV body frame y-axis

        # GPS controller
        # kp = 0.027  # 3*10**(-4) # 0.0812
        # ki = 0.00957
        # self.x_controller = pid(kp, ki, parent=self.parent)
        # self.y_controller = pid(kp, ki, parent=self.parent)

        # Measured manually
        # Roll and pitch changes all the time, feel free to edit them
        # #TODO: what is the concept behind these values? (could we let it hover and use an integrator to find them?)
        # self.steady_state_roll = 0.03090670969716989
        # self.steady_state_pitch = -0.016407850928032416
        # self.steady_state_thrust = -0.6603087868634927

        # Simulation values
        self.steady_state_roll = -0.015
        self.steady_state_pitch = -0.0183
        self.steady_state_thrust = -0.71
        self.forward_pitch = 0.01

        # The controller must run at this rate
        timer_period = 1/15
        self.update_loop = self.parent.create_timer(timer_period,
                                                    self.controller_update_loop)

        # Polar coordinates References
        self.refPolarPhi = 0  # [rad] ref. polar angle (angle to marker)
        self.refPolarDistance = 5  # [m] ref. polar radius (distance to marker)
        self.refAltitude = self.parent.takeoffAltitude  # [m] ref. UAV altitude

        self.setParameters()
        self.initFlags()

    def initFlags(self):
        # flags
        self.controllerEnable = True
        self.enableDistanceControl = False
        self.taking_off = True

    def markerDetectionHappened(self):
        return self.parent.camera.y is not None

    def altController(self):
        # Height controller
        thrust = self.height_controller.update(
            -np.abs(self.refAltitude), self.odometry.z)
        # Limit the thrust to max 10
        thrust = self.height_controller.limit(thrust, 10)
        # Normalize the thrust between 0 and 1
        thrust = (thrust-0)/(38 - 0)
        # Add the hover thurst
        thrust = self.steady_state_thrust + thrust

        return thrust
    
    def visualServoController(self):
        # Visual servo controller for target
        if self.parent.targetOffset is None:
            roll_correction = 0.0
        else:
            roll_correction = self.visual_servo_controller.update(self.parent.targetOffset,self.odometry.rx)
        # Limit roll_correction
        roll_correction = self.visual_servo_controller.limit(roll_correction, 0.05)
        return roll_correction

    def setParameters(self):
        self.openLoopPitch = 0.03
        # TODO: make it adpative

        self.markerXcorrectionRange = 10  # [m]

    # Loop
    def controller_update_loop(self):
        if not self.parent.odometry.is_odometry_recieved():
            return

        if not self.controllerEnable:
            print('Controller is not enabled')
            return

        thrust = self.altController()

        # When taking off the ground, simply try to maintain steady state roll and pitch
        if self.taking_off:
            self.parent.px4Handler.setAttitudeReference(
                self.steady_state_roll, self.steady_state_pitch, self.parent.homeHeading, thrust)
            return

        # # The marker based controller
        # if self.markerDetectionHappened():
        #     # The x position controller is only stable at short ranges

        #     roll = -self.y_cam_controller.update(0, self.parent.camera.y)
        #     roll += self.steady_state_roll

        #     if self.enableDistanceControl:
        #         if self.parent.camera.x < self.markerXcorrectionRange:
        #             # meaning of 6 and 0? is it pixels/inches in screen?
        #             cam_pitch = - \
        #                 self.x_cam_controller.update(
        #                     self.parent.refPolarDistance, self.parent.camera.x)
        #             pitch = cam_pitch + self.steady_state_pitch
        #         else:
        #             # TODO make a flag that controls if the controller should use the x position or raw pitch
        #             deltaDist = self.parent.refPolarDistance - self.parent.camera.x

        #             sign = 1 if deltaDist > 0 else -1
        #             pitch = self.steady_state_pitch - sign * self.openLoopPitch
        #     else:
        #         # use the steady state for now
        #         pitch = self.steady_state_pitch
        # else:
        #     roll = self.steady_state_roll

        #     if self.parent.camera.x < self.markerXcorrectionRange:
        #         # meaning of 6 and 0? is it pixels/inches in screen?
        #         cam_pitch = - \
        #             self.x_cam_controller.update(
        #                 self.parent.refPolarDistance, self.parent.camera.x)
        #         pitch = cam_pitch + self.steady_state_pitch
        #     else:
        #         pitch = self.steady_state_pitch

        bb_size = self.parent.bbsize
        if bb_size < self.parent.max_bbsize:
            pitch = self.steady_state_pitch - self.forward_pitch
        else:
            pitch = self.steady_state_pitch

        yaw = np.pi/2
        # roll = self.steady_state_roll
        roll = self.steady_state_roll + self.visualServoController()
        print(f"set roll: {roll}")
        
        self.parent.px4Handler.setAttitudeReference(roll, pitch, yaw, thrust)

    # This currently does nothing except setting the reference height
    def set_reference(self, x, y, z):
        self.refPolarDistance = x
        self.ref_y = y
        self.ref_z = z

    def setPolarReference(self, phi, radius, z):
        """
        Setting the reference position in polar coordinates for the marker localization controller.
        The convention is that the origin of the polar coordinate system is the camera.

        Args: #TODO: What is the frame that we give them?
            phi (float): [rad] ref. polar angle (angle to marker)
            radius (float): [m] ref. polar radius (distance to marker)
            z (float): [m] ref. UAV altitude

        """
        # TODO: do we need
        self.refPolarPhi = phi
        self.refPolarDistance = radius
        self.refAltitude = z
