import smach
import rospy
import subprocess


"""
Can't find any documentation for the pal_navigation_msgs/Acknowledge 
message type that the pal_navigation_sm service uses.
Docs are here: https://docs.pal-robotics.com/ari/sdk/23.1.13/services/pal_navigation_sm.html#service-pal-navigation-sm
And a 404 error for their message docs https://docs.ros.org/en/api/pal_navigation_msgs/html/service/Acknowledgment.html

For now, using a subprocess call to change between modes.

"""


class SwitchToMappingMode(smach.State):
    """
    State to switch the robot to mapping mode, as needed
    at the start of the restaurant task (as unmapped).
    """

    def __init__(self, navigation_service_name: str = "/pal_navigation_sm"):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._navigation_service_name = navigation_service_name
        rospy.wait_for_service(self._navigation_service_name)

    def execute(self, userdata):
        rospy.loginfo("Switching to mapping mode...")

        try:
            subprocess.call(
                ["rosservice", "call", self._navigation_service_name, "input: 'MAP'"]
            )
            rospy.loginfo("Switched to mapping mode successfully.")
            return "succeeded"
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to switch to mapping mode: {e}")
            return "failed"


class SwitchToLocMode(smach.State):
    """
    State to switch the robot to localization mode, as needed
    at the end of the restaurant task (as mapped).
    """

    def __init__(self, navigation_service_name: str = "/pal_navigation_sm"):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._navigation_service_name = navigation_service_name
        rospy.wait_for_service(self._navigation_service_name)

    def execute(self, userdata):
        rospy.loginfo("Switching to localization mode...")

        try:
            subprocess.call(
                ["rosservice", "call", self._navigation_service_name, "input: 'LOC'"]
            )
            rospy.loginfo("Switched to localization mode successfully.")
            return "succeeded"
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to switch to localization mode: {e}")
            return "failed"


class LoadMap(smach.State):
    """
    Load map on exit
    """

    def __init__(self, map_file_name: str = "2025-06-07-lab"):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._map_file = map_file_name

    def execute(self, userdata):
        rospy.loginfo(f"Loading map from {self._map_file}...")
        try:
            subprocess.call(
                [
                    "rosservice",
                    "call",
                    "/pal_map_manager/change_map",
                    f"input: '{self._map_file}'",
                ]
            )
            rospy.loginfo("Map loaded successfully.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to load map: {e}")
            return "failed"

        return "succeeded"
