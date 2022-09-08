#!/usr/bin/env python3

import rospy
from input_string_to_location.srv import StringToLocation, StringToLocationResponse
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus

def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    return Pose(Point(pose['position']['x'],
                      pose['position']['y'],
                      pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                           pose['orientation']['y'],
                           pose['orientation']['z'],
                           pose['orientation']['w']))


class StringToLocationSrv:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    
    def publish_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", pose.position.x, pose.position.y, pose.position.z)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_state()

        
    def __call__(self, req):
        print(req)
        resp = StringToLocationResponse()
        if get_pose_from_param(req.location.data):
            res = self.publish_goal(get_pose_from_param(req.location.data))
            if res == GoalStatus.SUCCEEDED:
                resp.is_reached.data = True
                return resp
            else:
                warn = f"Going to location failed with status: {res}"
                rospy.logwarn(warn)
                resp.is_reached.data = False
                return resp
        else:
            rospy.logwarn(" Incorrect pose ")
            resp.is_reached.data = False
            return resp


if __name__ == '__main__':
    rospy.init_node("string_to_location_server", anonymous=True)
    server = StringToLocationSrv()
    service = rospy.Service('/string_to_location', StringToLocation, server)
    rospy.loginfo("String to location server initialised")
    rospy.spin()


        