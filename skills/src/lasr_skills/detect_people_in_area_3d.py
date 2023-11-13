#!/usr/bin/env python3

from lasr_skills import DetectObjectsInArea3D

class DetectPeopleInArea3D(DetectObjectsInArea3D):

    def __init__(self):
        self.userdata.filter = ["person"]
        super().__init__()