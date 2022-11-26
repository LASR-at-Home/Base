#!/usr/bin/env python3
import rospy
from tiago_controllers.controllers import ArmController, TorsoController, BaseController, HeadController, \
    GripperController

class Controllers:
    """
        Class to merge all controllers together
    """

    def __init__(self):
        self.arm_controller = ArmController()
        self.torso_controller = TorsoController()
        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.gripper_controller = GripperController()
