#!/usr/bin/env python3

from lasr_skills import DetectObjects

class DetectPeople(DetectObjects):

    def __init__(self):
        super().__init__()
        self.userdata.filter = ["person"]