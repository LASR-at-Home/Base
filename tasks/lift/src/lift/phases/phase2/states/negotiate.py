#!/usr/bin/env python3
import smach
import rospy

from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
class Negotiate(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice

    def euclidian_distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        a = np.array((x1, y1))
        b = np.array((x2, y2))
        return np.linalg.norm(a - b)

    def get_how_close_to_door(self, min_dist=0.5):
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        door_position = get_pose_from_param("/door/pose")
        r = (robot_pose.position.x, robot_pose.position.y)
        d = (door_position.position.x, door_position.position.y)
        return self.euclidian_distance(r, d) < min_dist

    def rotate_to_face_door_new(self):
        """
        Rotate to face the door jareds
        """
        door_position = get_pose_from_param("/door/pose")
        rotation_angle = self.controllers.base_controller.compute_face_quat(door_position.position.x, door_position.position.y)
        self.controllers.base_controller.sync_to_pose(rotation_angle)

    def execute(self, userdata):
        # call and count the people objects
        self.voice.speak("Let's negotiate who is going out first")

        is_closer_to_door = self.get_how_close_to_door()
        if is_closer_to_door:
            self.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            self.voice.speak("I will wait by the lift for you.")
            result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/start/pose'))
            self.voice.speak("Should I wait more for you?")
            hear_wait = True
            if hear_wait:
                self.voice.speak("I will wait more")
                rospy.sleep(1)
            return 'success'

        else:
            self.voice.speak("I am not the closest to the door.")
            self.voice.speak("I will wait for you to exit first")
            rospy.sleep(1)
            # clear costmap
            clear_costmap()
            # maybe take the lift info again
            # if there are no clusters
            self.voice.speak("Exiting the lift")
            return 'success'
