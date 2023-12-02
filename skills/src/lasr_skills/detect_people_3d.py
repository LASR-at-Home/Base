#!/usr/bin/env python3

from lasr_skills import DetectObjects3D

class DetectPeople3D(DetectObjects3D):

    def __init__(self):
        super().__init__()
        self.userdata.filter = ["person"]