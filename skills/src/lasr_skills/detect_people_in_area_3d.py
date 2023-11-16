#!/usr/bin/env python3

from lasr_skills import DetectObjectsInArea3D

class DetectPeopleInArea3D(DetectObjectsInArea3D):

    def __init__(self):
        super().__init__()
        self.userdata.filter = ["person"]