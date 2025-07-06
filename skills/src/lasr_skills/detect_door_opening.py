#!/usr/bin/env python3
import smach
import time
import rospy
import numpy as np

from typing import Optional

from sensor_msgs.msg import LaserScan


class DetectDoorOpening(smach.State):
    """
    State to detect when a door has been opened.
    Assumes that the robot is directly facing the door, so once the
    LIDAR detects a change in distance, it is assumed
    that the door has been opened.
    """

    _scan_topic: str
    _scan_subscriber: rospy.Subscriber
    _door_opened: bool
    _opened_delta: float
    _timeout: float
    _initial_mean_distance: Optional[float]

    def __init__(
        self,
        lasr_scan_topic: str = "/scan",
        opened_delta: float = 0.5,
        timeout: float = 15.0,
    ):
        """State for checking if a door has been opened using LIDAR data.

        Args:
            lasr_scan_topic (str, optional): Topic to read the LIDAR data from.
            Defaults to "/scan".

            opened_delta (float, optional): Mean increase in distance in LIDAR
            reading to consider the door to be opened. Defaults to 0.5.

            timeout (float, optional): Maximum time to wait for the door to open.
            Defaults to 15.0 seconds. If this time has elapsed, assume that the door
            has been opened.
        """
        smach.State.__init__(self, outcomes=["door_opened"])

        self._scan_topic = lasr_scan_topic
        self._scan_subscriber = None
        self._door_opened = False
        self._opened_delta = opened_delta
        self._timeout = timeout
        self._initial_mean_distance = None

    def _is_door_opened(self, msg: LaserScan) -> None:
        """Checks if the door is opened by computing the mean
        distance from the laser scan data and comparing that to
        the mean distance from a scan reading of the closed door.

        Args:
            msg (LaserScan): The laser scan message containing distance data.
        """
        if self._door_opened:
            return
        range_data = np.array(msg.ranges)
        max_dist = msg.range_max
        min_dist = msg.range_min
        range_data[range_data == max_dist] = np.nan
        range_data[range_data == min_dist] = np.nan
        range_data[range_data == np.inf] = np.nan
        range_data[range_data == -np.inf] = np.nan
        mean_distance = np.nanmean(range_data)

        if self._initial_mean_distance is None:
            rospy.logwarn(
                "Initial mean distance not set. Cannot determine if door is opened."
            )
            return
        rospy.loginfo(f"Current mean distance: {mean_distance:.2f}")
        if mean_distance - self._initial_mean_distance > self._opened_delta:
            rospy.loginfo("Door has been opened.")
            self._door_opened = True

    def execute(self, userdata):
        rospy.loginfo("Waiting for door to open...")
        start_time = time.time()
        # Might need ot do filtering based on angles to narrow the FOV
        initial_scan = rospy.wait_for_message(self._scan_topic, LaserScan)
        max_dist = initial_scan.range_max
        min_dist = initial_scan.range_min

        # Message definition says that values of <range_max> or <range_min> should
        # be discarded, so set them to Nan
        initial_range_data = np.array(initial_scan.ranges)
        initial_range_data[initial_range_data == max_dist] = np.nan
        initial_range_data[initial_range_data == min_dist] = np.nan
        initial_range_data[initial_range_data == np.inf] = np.nan
        initial_range_data[initial_range_data == -np.inf] = np.nan

        self._initial_mean_distance = np.nanmean(initial_range_data)

        self._scan_subscriber = rospy.Subscriber(
            self._scan_topic,
            LaserScan,
            self._is_door_opened,
            queue_size=1,
        )

        while (not self._door_opened) or ((time.time() - start_time) < self._timeout):
            rospy.sleep(0.1)

        self._scan_subscriber.unregister()

        return "door_opened"


if __name__ == "__main__":
    rospy.init_node("detect_door_opening")
    detect = DetectDoorOpening()
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "DETECT_DOOR_OPENING",
            detect,
            transitions={"door_opened": "succeeded"},
        )
    sm.execute()
