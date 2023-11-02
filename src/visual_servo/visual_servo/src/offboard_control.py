#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


from .statemachines.stateMachine import StateMachine
from .statemachines import offboard_control_states
from .px4_interface.px4_handler import PX4Handler
from .controllers.controllers import position_controller
from .px4_interface.px4_odometry import px4_odometry
# from .camera.camera import camera
from .publishers.position_publishers import position_publishers

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from visual_servo_interfaces.msg import TuneController

_TARGET_SUB_TOPIC = "visual_servo/target_offset"

class OffboardControl(Node):
    def __init__(self):
        super().__init__('visual_servo')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.px4Handler = PX4Handler(self)

        self.odometry = px4_odometry(self)

        # self.camera = camera(self)

        self.initSubs()

        self.initPubs()

        self.setParameters()

        self.initVariables()

        self.stateMachineInit()
        self.stateMachineStart()

        self.position_controller = position_controller(self, self.odometry)

        # TMP
        self.start_pose_x = None
        self.start_pose_y = None

        self.position_publishers = position_publishers(self)

        self.tune_sub = self.create_subscription(
            TuneController, 'tune_controller', self.tune_callback,
            self.qos_profile)

    def tune_callback(self, msg):
        self.position_controller.steady_state_roll = msg.ss_roll
        self.position_controller.steady_state_pitch = msg.ss_pitch
        self.position_controller.steady_state_thrust = msg.ss_thrust
        self.position_controller.height_kp = msg.height_kp
        self.position_controller.thrust_limit = msg.thrust_limit
        self.takeoffAltitude = msg.ref_height
        print("New controller values received")

    # setup
    def initSubs(self):
        self.global_position = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehiclePositionCallback,
            self.qos_profile)

        self.polar_reference_sub = self.create_subscription(
            PoseStamped, '/polarRef', self.polarReferenceCallback,
            self.qos_profile)

        self.target_sub = self.create_subscription(
            Vector3Stamped, _TARGET_SUB_TOPIC, self.targetFrameCallback, self.qos_profile)

    def initPubs(self):
        pass

    def initVariables(self):
        # flags TODO: make this a class
        self.mission_started = False
        self.mission_complete = False
        self.landingFlag = False

        self.homeHeading = None
        # [rad] current heading of the UAV from compass
        self.currentHeading = None

        # [m] ref. polar radius (distance to marker)
        self.refPolarDistance = None
        self.refPolarPhi = None  # [rad] ref. polar angle (angle to marker)
        self.refAltitude = None  # [m] ref. UAV altitude

        self.targetOffset_x = None  # [-1,1] relative position of target in frame
        self.targetOffset_y = None
        self.bbsize = None      # proportion of bb of frame
        self.max_bbsize = 0.5   # bbsize to terminate mission

    def setParameters(self):
        self.dt = 0.2
        # all altitude values are in [m] and must be negative (NED frame)
        # [m] The altitude at which the drone is considered to be landed
        self.landedAltitude = 0.2
        # [m] The altitude at which the drone is considself.take_offered to be taken off
        self.takeoffAltitude = 2.5

        # # Camera estimation
        # self.camera_x = None
        # self.camera_y = None
        # self.camera_z = None

    # callbacks
    def vehiclePositionCallback(self, msg: VehicleLocalPosition):
        """Vehicle position callback from px4.

        Args:
            msg (VehicleLocalPosition): The position of the drone in NED frame (WARNING!!!).
        """

        if self.homeHeading is None:
            self.homeHeading = msg.heading

        self.currentHeading = msg.heading

    def polarReferenceCallback(self, msg: PoseStamped):
        """Polar reference callback. 
        The convention is that the origin of the polar coordinate system is the camera.
        The msg.x corresponds to the radius, msg.y corresponds to the angle and msg.z corresponds to the altitude.

                UAV  
            |    /
            |   / 
            |  / msg.y -> Ref. distance to marker
            | /
            |/  msg.x -> Ref. angle to marker (in radians)
           camera

        Args:
            msg (PoseStamped): The position of the marker in polar coordinates.
            msg.x [m]   -> radius (distance to marker)
            msg.y [rad] -> angle (angle to marker)
            msg.z [m]   -> UAV altitude with respect to the ground (or marker?) 
        """
        self.refPolarDistance = msg.pose.position.x
        self.refPolarPhi = msg.pose.position.y
        self.refAltitude = msg.pose.position.z

    def targetFrameCallback(self, msg: Vector3Stamped):
        # self.targetOffset_x = msg.x
        # self.targetOffset_y = msg.y
        # self.bbsize = msg.z

        self.targetOffset_x = msg.vector.x
        self.get_logger().info(f'x offset {self.targetOffset_x}')
        self.targetOffset_y = msg.vector.y
        self.bbsize = msg.vector.z

    # status manual checks
    def isTakenOff(self):
        print("self.odemetry.z: ", self.odometry.z)
        return abs(self.odometry.z) >= self.takeoffAltitude * 0.85

    def isLanded(self):
        return abs(self.odometry.z) <= self.landedAltitude

    def hasReceivedNewReference(self):
        # raise NotImplementedError
        return self.refPolarDistance is not None and self.refPolarPhi is not None and self.refAltitude is not None

    # state machine functionalities
    def stateMachineInit(self) -> None:
        self.stateMachine = StateMachine()
        self.stateMachine.loadStates([
            offboard_control_states.PRE_OFFBOARD,
            offboard_control_states.TAKING_OFF,
            offboard_control_states.HOVERING,
            offboard_control_states.SETTING_YAW
        ], self)

        self.stateMachine.stateTransit(offboard_control_states.PRE_OFFBOARD)

    def stateMachineStart(self) -> None:
        freq = 10  # [Hz]
        self.stateMachineTimer = self.create_timer(
            1/freq, self.stateMachineCallback)

    def stateMachineCallback(self) -> None:
        self.stateMachine.stateIterate()
