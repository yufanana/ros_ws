from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..offboard_control import OffboardControl

from .pid_controllers import pid
import numpy as np
import time


class position_controller():
    def __init__(self, parent, odometry_class):
        self.parent: OffboardControl = parent

        self.odometry = odometry_class

        # Simulation values for Iris
        self.steady_state_roll = -0.015
        self.steady_state_pitch = -0.0183
        self.steady_state_thrust = -0.71
        self.forward_pitch = 0.01
        self.thrust_limit = 10.0
        # Height controller's negavtive values means up
        self.height_controller = pid(kp=7.08, ki=0.0022, a1=-0.5, b0=7.75, b1=-7.25,
                                     gain=1, integral_limit=5, parent=self.parent)
        # Visual servo controller
        self.vservo_x_controller = pid(
            kp=0.2, gain=0.5, integral_limit=5, parent=self.parent)
        self.vservo_y_controller = pid(
            kp=0.2, gain=0.5, integral_limit=5, parent=self.parent)

        # # Simulation values for Typhoon H480
        # self.steady_state_roll = -0.015
        # self.steady_state_pitch = -0.0183
        # self.steady_state_thrust = -0.4
        # self.forward_pitch = 0.01
        # self.thrust_limit = 8.0
        # # Height controller's negavtive values means up
        # self.height_controller = pid(kp=3.0, ki=0.003, a1=-0.5, b0=7.75, b1=-7.25,
        #                              gain=1, integral_limit=5, parent=self.parent)

        # # Visual servo controller
        # self.vservo_x_controller = pid(kp=0.2, ki=0.003, gain=0.4, integral_limit=5, parent=self.parent)
        # self.vservo_y_controller = pid(kp=0.2, ki=0.003, gain=0.4, integral_limit=5, parent=self.parent)

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

    def setParameters(self):
        # self.openLoopPitch = 0.03
        # TODO: make it adaptive
        # self.markerXcorrectionRange = 10  # [m]
        pass

    def markerDetectionHappened(self):
        return self.parent.camera.y is not None

    def targetDetectionHappened(self):
        return self.parent.targetOffset_x is not None

    def altController(self):
        # Height controller
        thrust = self.height_controller.update(
            -np.abs(self.refAltitude), self.odometry.z)

        # Limit the thrust to max 10 for Iris
        thrust = self.height_controller.limit(thrust, 10)

        # Limit the thrust to max for Typhoon H480
        # thrust = self.height_controller.limit(thrust, 8)

        thrust = self.height_controller.limit(thrust, self.thrust_limit)
        # Normalize the thrust between 0 and 1
        thrust = (thrust-0)/(38 - 0)
        # Add the hover thurst
        thrust = self.steady_state_thrust + thrust
        return thrust

    def vservo_x_Controller(self):
        # Visual servo controller for target
        if self.parent.targetOffset_x is None:
            roll_correction = 0.0
        else:
            roll_correction = self.vservo_x_controller.update(
                self.parent.targetOffset_x, self.odometry.rx)
        # Limit roll_correction
        roll_correction = self.vservo_x_controller.limit(roll_correction, 0.05)
        return roll_correction

    def vservo_y_Controller(self):
        # Visual servo controller for target
        if self.parent.targetOffset_y is None:
            pitch_correction = 0.0
        else:
            pitch_correction = self.vservo_y_controller.update(
                self.parent.targetOffset_x, self.odometry.ry)
        # Limit pitch correction
        pitch_correction = self.vservo_y_controller.limit(
            pitch_correction, 0.05)
        return pitch_correction

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

        roll = self.steady_state_roll
        pitch = self.steady_state_pitch
        yaw = np.pi/2

        if self.targetDetectionHappened():
            # print("x: {0} bbsize: {1}".format(self.parent.targetOffset_x,self.parent.bbsize))
            if self.parent.bbsize < self.parent.max_bbsize:
                pitch -= self.forward_pitch
            roll += self.vservo_x_Controller()
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
