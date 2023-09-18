#!/usr/bin/env python3
import rospy
import requests
from std_msgs.msg import Int16, Empty

rospy.init_node("datahub_shit")

DATAHUB_API = "https://ecs-mnemosyne.azurewebsites.net/api/Hub"
ROBOT_ID = "019b1442-728d-48c4-1de7-08dbb68bfcf1"

def start_episode(_):
    requests.post(DATAHUB_API, json={"RobotId" : ROBOT_ID, "Competition" : "ERL", "Action" : "STARTEPISODE", "Episode" : 1})

def stop_episode(_):
    requests.post(DATAHUB_API, json={"RobotId" : ROBOT_ID, "Competition" : "ERL", "Action" : "STOPEPISODE", "Episode" : 1})

def start_phase(msg):
    requests.post(DATAHUB_API, json={"RobotId" : ROBOT_ID, "Competition" : "ERL", "Action" : "STARTPHASE", "Episode" : 1, "Phase" : msg.data})

def stop_phase(msg):
    requests.post(DATAHUB_API, json={"RobotId" : ROBOT_ID, "Competition" : "ERL", "Action" : "STARTPHASE", "Episode" : 1, "Phase" : msg.data})

def ping(_):
    requests.post(DATAHUB_API, json={"RobotId" : ROBOT_ID, "Competition" : "ERL", "Action" : "PING"})

rospy.Subscriber("/datahub/start_episode", Empty, start_episode)
rospy.Subscriber("/datahub/stop_episode", Empty, stop_episode)
rospy.Subscriber("/datahub/start_phase", Int16, start_phase)
rospy.Subscriber("/datahub/stop_phase", Int16, stop_phase)
rospy.Subscriber("/datahub/ping", Int16, ping)

rospy.spin()