#!/usr/bin/env python3
import numpy as np
import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
import json

class ScheduleGoingOut(smach.State):
    def __init__(self, controllers, voice, speech):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice
        self.controllers = controllers
        self.speech = speech

    # import this from base planner
    def euclidian_distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        a = np.array((x1, y1))
        b = np.array((x2, y2))
        return np.linalg.norm(a - b)

    def get_how_close_to_door(self,is_robot, min_dist=0.5):
        dist = self.get_dist_to_door(is_robot)
        return round(dist, 1) < min_dist
    def get_dist_to_door(self, is_robot, x=None, y=None):
        if is_robot:
            robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
            print(f"robot pose: {robot_pose}")
            r = (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
        else:
            r = (x, y)

        door_position = get_pose_from_param("/door/pose")
        print(f"door pose: {door_position}")
        d = (door_position.position.x, door_position.position.y)

        dist = self.euclidian_distance(r, d)
        print(f"distance to door: {dist}")
        return dist


    def rotate_to_face_door_new(self):
        """
        Rotate to face the door jareds
        """
        door_position = get_pose_from_param("/door/pose")
        rotation_angle = self.controllers.base_controller.compute_face_quat(door_position.position.x,
                                                                            door_position.position.y)
        self.controllers.base_controller.sync_to_pose(rotation_angle)

    def create_point_marker(self, x, y, z, idx, color=(0, 1, 0), text="none"):
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.id = idx
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.color.r = color[0]
        marker_msg.color.g = color[1]
        marker_msg.color.b = color[2]
        marker_msg.text = text
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = z
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        return marker_msg
    def get_random_rgb(self):
        import random
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        print(f"Random RGB: ({r}, {g}, {b})")
        return (r, g, b), "This is {}, {}, {}".format(r, g, b)

    def rank(self):
        centers = rospy.get_param("/lift/centers")
        # get the distance of each center to the door
        distances = []
        for center in centers:
            distances.append((self.get_dist_to_door(False, center[0], center[1]), center))
        # rank the distances (dist, center-person) and add the robot
        distances.append((self.get_dist_to_door(True), (0, 0)))
        # sort the distances
        sorted_distances = sorted(distances, key=lambda x: x[0])
        print("sorted distances")
        print(sorted_distances)
        random_colour, random_text = self.get_random_rgb()
        people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)
        for i, dist in enumerate(sorted_distances):
            mk = self.create_point_marker(dist[1][0], dist[1][1], 0, i, random_colour, random_text)
            people_pose_pub.publish(mk)


        # mk = self.create_point_marker(sorted_distances[0][1][0], sorted_distances[0][1][1], 0, 0, random_colour, random_text)
        # people_pose_pub.publish(mk)
        # oif the robot is closest or second to closest return true
        if sorted_distances[0][1] == (0, 0) or sorted_distances[1][1] == (0, 0):
            return True
        else:
            return False

    def listen(self):
        resp = self.speech()
        if not resp.success:
            self.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def hear_wait(self):
        resp = self.listen()

        if resp["intent"]["name"] == "negotiate_lift":
            # I'm going to wait
            wait = resp["entities"].get("wait_command", [])
            if not wait:
                self.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_wait()
            else:
                return True
        else:

            return False


    # the door is open here!!!
    def execute(self, userdata):
        self.voice.speak("I know there are {} people in the lift".format(rospy.get_param("/lift/num_clusters")))

        is_robot_closest_rank = self.rank()
        print("is robot closest rank->>> {}".format(is_robot_closest_rank))

        is_closer_to_door = is_robot_closest_rank
        if is_closer_to_door:
            self.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            state = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre/pose'))
            if not state:
                return 'failed'
            # turn around
            self.voice.speak("Just minding my own business!")
            self.controllers.base_controller.rotate_to_face_object(object_name='/door/pose')
            # wait for the person to exit
            rospy.sleep(2)
            self.voice.speak("Should I wait more for you?")
            # hear
            hear_wait = True
            count = 0
            while hear_wait and count < 5:
                if RASA:
                    hear_wait = self.hear_wait()
                    if hear_wait:
                        self.voice.speak("I will wait more")
                        rospy.sleep(5)
                    else:
                        self.voice.speak("i am done with waiting")
                        break
                    count += 1
            return 'success'
        else:
            self.voice.speak("I am not the closest to the door.")
            self.voice.speak("I will wait inside the lift because this is not my floor")
            return 'success'
