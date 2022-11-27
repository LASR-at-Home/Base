#!/usr/bin/env python3
from tiago_controllers.controllers import BaseController, HeadController

class Controllers:
    """
        Class to merge all controllers together
    """

    def __init__(self):
        self.base_controller = BaseController()
        self.head_controller = HeadController()