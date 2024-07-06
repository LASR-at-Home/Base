#!/usr/bin/env python3
import rospy
import smach
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lasr_skills.vision import GetImage


class TorsoHeight(smach.State):
    def __init__(self, target_height, duration):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.target_height = target_height
        self.duration = duration

    def execute(self, userdata):
        rospy.loginfo("Adjusting torso height to %s meters", self.target_height)
        pub = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=10, latch=True
        )
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ["torso_lift_joint"]

        point = JointTrajectoryPoint()
        point.positions = [self.target_height]
        # point.time_from_start.nsecs = 1000000
        trajectory.points = [point]

        try:
            # Rotate for the specified duration
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < self.duration:
                # Publish the command
                print(trajectory)
                pub.publish(trajectory)
                rospy.sleep(0.1)
            return (
                "succeeded"  # Wait for the duration of the movement plus a little extra
            )
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt Exception")
            return "failed"


if __name__ == "__main__":
    rospy.init_node("adjust_torso")
    while not rospy.is_shutdown():
        sm = TorsoHeight(-0.1, 1)
        sm.execute()
    rospy.spin()
