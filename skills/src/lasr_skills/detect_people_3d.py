#!/usr/bin/env python3

from lasr_skills import DetectObjects3D

class DetectPeople3D(DetectObjects3D):

    def __init__(self):
        self.userdata.filter = ["person"]
        super().__init__()