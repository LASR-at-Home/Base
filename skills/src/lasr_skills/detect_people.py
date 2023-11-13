#!/usr/bin/env python3

from lasr_skills import DetectObjects

class DetectPeople(DetectObjects):

    def __init__(self):
        self.userdata.filter = ["person"]
        super().__init__()