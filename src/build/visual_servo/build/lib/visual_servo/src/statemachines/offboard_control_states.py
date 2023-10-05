#!/usr/bin/env python
from __future__ import annotations
import numpy as np
from std_msgs.msg import Bool, String, Int16
import time
from .stateMachine import State
from .stateMachine import StateMachine
from typing import TYPE_CHECKING

__author__ = "Marios Nektarios Stamatopoulos"
__contact__ = "marsta@ltu.se"

if TYPE_CHECKING:
    from ..offboard_control import OffboardControl


# TODO
# A lot of these states kinda do nothing
# Only PRE_OFFBOARD and TAKING_OFF actually does anything "state like"


class PRE_OFFBOARD(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("PRE_OFFBOARD", parentnObj)
        self.parentObj: OffboardControl  # set type hinting for parentObj

        self.start_time = time.time()

    def iterate(self) -> None:
        if self.parentObj.homeHeading is None:
            return
        # spam messages for 2 seconds
        dt = time.time() - self.start_time
        if dt <= 2:
            self.parentObj.px4Handler.attitude_mode()
        else:
            print("Pre-offboard done ... Transiting to TAKING_OFF...")
            self.parentObj.px4Handler.arm()
            self.parentObj.px4Handler.offboard()
            self.transit(TAKING_OFF)


class TAKING_OFF(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("TAKING_OFF", parentnObj)
        self.parentObj: OffboardControl  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.px4Handler.attitude_mode()
        self.parentObj.position_controller.controllerEnable = True

        if self.parentObj.isTakenOff():
            print("UAV took off ... Transiting to RESET_YAW")
            self.parentObj.position_controller.taking_off = False
            self.transit(HOVERING)
        else:
            self.parentObj.position_controller.taking_off = True

        self.parentObj.position_controller.set_reference(
            self.parentObj.homeHeading, 5, -self.parentObj.takeoffAltitude)


class HOVERING(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("HOVERING", parentnObj)
        self.parentObj: OffboardControl
        self.start_time = None

    def iterate(self) -> None:
        self.parentObj.px4Handler.attitude_mode()

        self.parentObj.position_controller.controllerEnable = True
        self.parentObj.position_controller.enableDistanceControl = False

        if self.parentObj.hasReceivedNewReference():
            self.transit(SETTING_YAW)


class SETTING_YAW(State):  # TODO: Why do we need this state?
    def __init__(self, parentnObj) -> None:
        super().__init__("SETTING_YAW", parentnObj)
        self.parentObj: OffboardControl  # set type hinting for parentObj

        self.currDistance = None
        self.currentAltitude = None
        self.initialheading = None

        self.angleErrorThreshold = np.deg2rad(10)
        self.angularSpeed = np.deg2rad(8)

    def iterate(self) -> None:
        self.parentObj.px4Handler.attitude_mode()
        self.parentObj.position_controller.enableDistanceControl = False

        # fix the distance and altitude to a constant value
        if self.currDistance is None:
            self.currDistance = self.parentObj.camera.x  # TODO: check that it is correct

        if self.currentAltitude is None:
            self.currentAltitude = self.parentObj.odometry.z  # TODO: check that it is correct

        if self.initialheading is None:
            self.initialheading = self.parentObj.currentHeading - self.parentObj.homeHeading
            self.t0 = time.time()
            self.dS = self.parentObj.refPolarPhi - self.initialheading
            self.speedSign = np.sign(self.dS)

            self.duration = np.abs(self.dS) / self.angularSpeed

        dt = time.time() - self.t0
        if dt > self.duration:
            dt = self.duration

        phiRef = self.parentObj.homeHeading + self.initialheading + \
            self.speedSign * self.angularSpeed * dt

        self.parentObj.position_controller.setPolarReference(
            phiRef, self.currDistance, self.currentAltitude)

        # calculate angle error
        thetaError = np.abs(self.parentObj.refPolarPhi -
                            self.parentObj.currentHeading)
        if thetaError < self.angleErrorThreshold:
            self.transit(SETTING_DISTANCE)


class SETTING_DISTANCE:
    def __init__(self, parentnObj) -> None:
        super().__init__("SETTING_DISTANCE", parentnObj)
        self.parentObj: OffboardControl

        self.currentAltitude = None
        self.currentPhi = None

    def iterate(self) -> None:
        self.parentObj.px4Handler.attitude_mode()
        self.parentObj.position_controller.enableDistanceControl = True

        if self.currentPhi is None:
            self.currentPhi = self.parentObj.currentHeading

        self.parentObj.position_controller.setPolarReference(
            self.currentPhi, self.parentObj.refPolarDistance, self.parentObj.refAltitude)

        deltaDist = np.abs(self.parentObj.refPolarDistance -
                           self.parentObj.camera.x)

        if deltaDist < 0.5:
            self.transit(HOVERING)
