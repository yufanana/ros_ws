from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..offboard_control import OffboardControl

from .pid_controllers import pid
import numpy as np


class position_controller():
    def __init__(self, parent, odometry_class):
        self.parent: OffboardControl = parent

        self.init_flags()

        self.init_controllers()
        
        self.init_steady_state_values()

        self.odometry = odometry_class

        # The controller must run at this rate
        timer_period = 1/15
        self.update_loop = self.parent.create_timer(timer_period,
                                                    self.controller_update_loop)

        self.refAltitude = self.parent.takeoffAltitude  # [m] ref. UAV altitude

        # Polar coordinates References
        self.refPolarPhi = 0 # [rad] ref. polar angle (angle to marker)

    def init_flags(self):
        self.controllerEnable = True
        self.taking_off = True
        self.first_detection = False
        self.reached_target = False

    def init_controllers(self):
        ###### Simulation values for x500
        # Height controller's negative values means up
        self.height_controller = pid(kp=7.08, ki=0.0022, a1=-0.5, b0=7.75, b1=-7.25,
                                     gain=1, integral_limit=5, parent=self.parent)
        # Visual servo controller
        self.vservo_x_controller = pid(
            kp=0.3, gain=0.1, integral_limit=5, parent=self.parent)
        self.vservo_y_controller = pid(
            kp=0.3, gain=0.1, integral_limit=5, parent=self.parent)
        
        # Position controllers
        kp = 0.027 #  3*10**(-4) # 0.0812
        ki = 0.00957
        self.x_controller = pid(kp, ki, parent=self.parent)


        ###### Simulation values for Iris
        # # Height controller's negative values means up
        # self.height_controller = pid(kp=7.08, ki=0.0022, a1=-0.5, b0=7.75, b1=-7.25,
        #                              gain=1, integral_limit=5, parent=self.parent)
        # # Visual servo controller
        # self.vservo_x_controller = pid(
        #     kp=0.2, gain=0.5, integral_limit=5, parent=self.parent)
        # self.vservo_y_controller = pid(
        #     kp=0.2, gain=0.5, integral_limit=5, parent=self.parent)

        ###### Simulation values for Typhoon H480
        # # Height controller's negative values means up
        # self.height_controller = pid(kp=3.0, ki=0.003, a1=-0.5, b0=7.75, b1=-7.25,
        #                              gain=1, integral_limit=5, parent=self.parent)
        # # Visual servo controller
        # self.vservo_x_controller = pid(kp=0.2, ki=0.003, gain=0.4, integral_limit=5, parent=self.parent)
        # self.vservo_y_controller = pid(kp=0.2, ki=0.003, gain=0.4, integral_limit=5, parent=self.parent)

    def init_steady_state_values(self):
         # Simulation values for x500
        self.steady_state_roll = 0.0
        self.steady_state_pitch = 0.0
        self.steady_state_thrust = -0.71       # for simulation
        # self.steady_state_thrust = -0.660   # for real drone 
        self.forward_pitch = 0.00
        self.thrust_limit = 10.0
        self.offset_age = 0
        self.max_age = 10

        # # Simulation values for Iris
        # self.steady_state_roll = -0.015
        # self.steady_state_pitch = -0.0183
        # self.steady_state_thrust = -0.71
        # self.forward_pitch = 0.01
        # self.thrust_limit = 10.0
        # self.offset_age = 0
        # self.max_age = 20

        # # Simulation values for Typhoon H480
        # self.steady_state_roll = -0.015
        # self.steady_state_pitch = -0.0183
        # self.steady_state_thrust = -0.4
        # self.forward_pitch = 0.01
        # self.thrust_limit = 8.0
      

    def targetDetectionHappened(self):
        if self.parent.targetOffset_x is not None:
            self.first_detection = True
        return self.first_detection

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
    
    def x_Controller(self):
        """
        Position controller for x position using GPS during testing.
        """
        # TODO: what do I do with the output of controller? 
        x = self.x_controller.update(0, self.odometry.x)
        
        return x


    def vservo_x_Controller(self):
        """
        Visual servo to control the roll of the drone.
        """
        if self.parent.targetOffset_x is None:
            roll_correction = 0.0
        else:
            roll_correction = self.vservo_x_controller.update(
                self.parent.targetOffset_x, self.odometry.rx)

        # Limit roll_correction
        roll_correction = self.vservo_x_controller.limit(roll_correction, 0.05)
        
        # Reset variable to wait for new value
        self.parent.targetOffset_x = None

        return roll_correction

    def vservo_y_Controller(self):
        """
        Used to control pitch of camera/drone. Currently not in use.
        """
        if self.parent.targetOffset_y is None:
            pitch_correction = 0.0
        else:
            pitch_correction = self.vservo_y_controller.update(
                self.parent.targetOffset_x, self.odometry.ry)

        # Limit pitch correction
        pitch_correction = self.vservo_y_controller.limit(pitch_correction, 0.05)

        # Reset variable to wait for new value
        self.parent.targetOffset_y = None
        
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

        # check if first_detection has occured
        if self.targetDetectionHappened():

            # increment offset_age if no new offset value is received
            if self.parent.targetOffset_x is None:
                self.offset_age += 1
                if self.offset_age > self.max_age:
                    pitch = self.steady_state_pitch
        
            # use offset value from YOLO detection
            else:
                if self.parent.bbsize < self.parent.max_bbsize:
                    self.offset_age = 0
                    pitch -= self.forward_pitch
                    roll += self.vservo_x_Controller()
                else:
                    self.reached_target = True
                    self.parent.px4Handler.land()
                    self.parent.get_logger().info("Reached target, landing...", once=True)
        # TODO: add landing command once the drone has reached the target
        
        
        print(f"set roll: {roll}")
        self.parent.px4Handler.setAttitudeReference(roll, pitch, yaw, thrust)

        # GPS position controller for testing, not sure
        # Do i have to set drone offboard mode to be in position mode? in px4_handler.py
        x = self.x_Controller()
        y = 0
        z = self.parent.takeoffAltitude
        self.parent.px4Handler.setPositionReference([x, y, z])

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
