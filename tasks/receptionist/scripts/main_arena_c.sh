#!/bin/bash

# Change map
rosservice call /pal_map_manager/change_map "input: 'robocup_2024_onlyOPL'"
# launch setup.launch, with correct config pases
    roslaunch receptionist setup.launch config:=robocup_opl
# run main.py
rosrun receptionist main.py