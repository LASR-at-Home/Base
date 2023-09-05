#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
import yaml

class PoseRecorder:
    def __init__(self):
        self.points = []
        self.num_points = 0
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PoseStamped, self.clicked_point_callback)
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.amcl_pose_callback)

    def clicked_point_callback(self, msg):
        self.points.append({'pose': {'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
                                     'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                                                     'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}}})
        self.num_points += 1

    def amcl_pose_callback(self, msg):
        if self.num_points > 0:
            self.points[-1]['name'] = 'point{}'.format(self.num_points)

    def save_to_yaml(self, request):
        if self.num_points == 0:
            return EmptyResponse()

        data = {'points': self.points}

        with open('output.yaml', 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=False)

        rospy.loginfo('Saved {} points to output.yaml'.format(self.num_points))
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('pose_recorder')
    pose_recorder = PoseRecorder()

    save_service = rospy.Service('save_pose_to_yaml', Empty, pose_recorder.save_to_yaml)

    rospy.spin()
