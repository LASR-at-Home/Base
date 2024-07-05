#!/usr/bin/env python3
import rospy
import smach
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AdjustHeadTilt(smach.State):
    def __init__(self, tilt, duration):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.tilt = tilt
        self.duration = duration
        self.pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

    def execute(self):
        if not rospy.is_shutdown():
            rospy.loginfo('Adjusting head tilt to %s radians', self.tilt)
            trajectory = JointTrajectory()
            trajectory.header.stamp = rospy.Time.now()
            trajectory.joint_names = ["head_2_joint"]

            point = JointTrajectoryPoint()
            point.positions = [self.tilt]
            point.time_from_start = rospy.Duration(self.duration)
            trajectory.points = [point]

            start_time = rospy.Time.now()
            rate = rospy.Rate(50)  # Use a higher frequency for trajectory commands

            while (rospy.Time.now() - start_time).to_sec() < self.duration:
                if rospy.is_shutdown():
                    rospy.loginfo("ROS node shutdown detected")
                    return 'failed'
                self.pub.publish(trajectory)
                rate.sleep()
            
            return 'succeeded'
        else:
            rospy.loginfo("ROS node shutdown detected")
            return 'failed'


if __name__ == '__main__':
    rospy.init_node('adjust_head')
    # heads up--> 0.1 
    # heads down --> -0,1
    sm = AdjustHeadTilt(0.1, 1)
    outcome = sm.execute()
    rospy.spin()