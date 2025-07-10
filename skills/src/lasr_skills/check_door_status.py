#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from lasr_skills import Say
from sklearn.cluster import KMeans


class CheckDoorStatus(smach.State):
    def __init__(
        self,
        expected_closed_depth=None,
        roi=None,
        change_thresh=0.5,
        open_thresh=0.6,
        score_thresh=1.0,
    ):
        smach.State.__init__(self, outcomes=["open", "closed", "error"])
        self.expected_depth = expected_closed_depth  # trusted, map-based depth
        self.change_thresh = (
            change_thresh  # how much observed baseline can differ from expected
        )
        self.open_thresh = open_thresh  # how much deeper to consider door open
        self.score_thresh = (
            score_thresh  # minimum weighted score to confirm door is open
        )

        self.roi = roi

        self.lock = threading.Lock()
        self.latest_cloud = None
        self.ready = False

        self.sub = rospy.Subscriber(
            "/xtion/depth_pointsros", PointCloud2, self.callback
        )

    def callback(self, msg):
        with self.lock:
            self.latest_cloud = msg
            self.ready = True

    def get_roi_data(self):
        with self.lock:
            if self.latest_cloud is None:
                return None, "no_data"
            try:
                cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
                    self.latest_cloud, remove_nans=True
                )

                if self.roi is None:
                    self.roi = self.auto_roi(cloud)

                mask = (
                    (cloud[:, 0] > self.roi["x"][0])
                    & (cloud[:, 0] < self.roi["x"][1])
                    & (cloud[:, 1] > self.roi["y"][0])
                    & (cloud[:, 1] < self.roi["y"][1])
                )
                roi_points = cloud[mask]

                if roi_points.shape[0] == 0:
                    return None, "empty"

                return roi_points[:, 2], "ok"

            except Exception as e:
                rospy.logerr("Point cloud processing error: %s", e)
                return None, "error"

    def auto_roi(self, cloud):
        x_min, x_max = np.percentile(cloud[:, 0], [40, 60])
        y_min, y_max = np.percentile(cloud[:, 1], [40, 60])
        return {"x": (x_min, x_max), "y": (y_min, y_max), "z": (0.3, 4.0)}

    def analyze_depth(self, depth_data):
        if depth_data is None or len(depth_data) == 0:
            return None

        sorted_data = np.sort(depth_data)
        avg = np.median(sorted_data)
        variance = np.var(sorted_data)
        gradient = np.mean(np.abs(np.gradient(sorted_data)))
        edge_drop = np.max(np.abs(np.diff(sorted_data))) > 0.5
        multi_cluster = self.cluster_depths(sorted_data)

        rospy.loginfo(
            "Avg: %.3f, Var: %.4f, Grad: %.4f, EdgeDrop: %s, MultiCluster: %s",
            avg,
            variance,
            gradient,
            edge_drop,
            multi_cluster,
        )

        return {
            "avg": avg,
            "variance": variance,
            "gradient": gradient,
            "edge_drop": edge_drop,
            "multi_cluster": multi_cluster,
        }

    def cluster_depths(self, depth_data):
        try:
            kmeans = KMeans(n_clusters=2, n_init="auto").fit(depth_data.reshape(-1, 1))
            centers = np.sort(kmeans.cluster_centers_.flatten())
            return abs(centers[1] - centers[0]) > 0.3
        except Exception:
            return False

    def capture_baseline(self, duration=1.0):
        rospy.loginfo("Capturing baseline for %.1f seconds...", duration)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        collected = []
        rate = rospy.Rate(10)
        while rospy.Time.now() < end_time:
            data, status = self.get_roi_data()
            if data is not None and status == "ok":
                collected.extend(data.tolist())
            rate.sleep()
        return np.array(collected) if collected else None

    def execute(self, userdata):
        rospy.loginfo("Waiting for depth reading...")
        self.ready = False
        self.latest_cloud = None

        timeout = rospy.Time.now() + rospy.Duration(5)
        rate = rospy.Rate(5)
        while not self.ready and rospy.Time.now() < timeout:
            rate.sleep()

        observed_baseline = self.capture_baseline(1.0)
        if observed_baseline is None:
            return "error"

        observed_result = self.analyze_depth(observed_baseline)
        if observed_result is None:
            return "error"

        observed_avg = observed_result["avg"]

        if self.expected_depth is not None:
            if observed_avg > self.expected_depth + self.change_thresh:
                rospy.logwarn(
                    "Observed baseline is deeper than expected — validating with structure..."
                )
                if observed_result["edge_drop"] or observed_result["multi_cluster"]:
                    rospy.logwarn("Also found structural cues — likely open")
                    return "open"
                else:
                    rospy.loginfo(
                        "Deeper but structurally consistent — trusting as closed"
                    )
            else:
                rospy.loginfo("Baseline matches expected — trusting it")
        else:
            self.expected_depth = observed_avg

        depth_data, status = self.get_roi_data()
        if status in ["no_data", "error"]:
            return "error"
        elif status == "empty":
            return "closed"

        result = self.analyze_depth(depth_data)
        if result is None:
            return "error"

        open_score = 0.0

        if result["avg"] > self.expected_depth + self.open_thresh:
            open_score += 2.0

        if result["multi_cluster"]:
            open_score += 1.5

        if result["edge_drop"]:
            open_score += 1.0

        rospy.loginfo(
            "Open score: %.2f (threshold: %.2f)", open_score, self.score_thresh
        )

        if open_score >= self.score_thresh:
            return "open"

        if result["gradient"] < 0.03 and result["variance"] < 0.01:
            return "closed"

        return "closed"


def main():
    rospy.init_node("door_status_state_machine")
    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])

    with sm:
        smach.StateMachine.add(
            "CHECK_DOOR_STATUS",
            CheckDoorStatus(
                expected_closed_depth=2,
                change_thresh=0.4,
                open_thresh=0.6,
                score_thresh=1.0,
            ),
            transitions={
                "open": "SAY_DOOR_STATUS",
                "closed": "DONE",
                "no_object": "FAILED",
                "error": "FAILED",
            },
        )

        smach.StateMachine.add(
            "SAY_DOOR_STATUS",
            Say(text="Door is open"),
            transitions={
                "succeeded": f"DONE",
                "aborted": f"DONE",
                "preempted": f"DONE",
            },
        )

    smach_viewer = smach_ros.IntrospectionServer("viewer", sm, "/SM_ROOT")
    smach_viewer.start()
    outcome = sm.execute()
    rospy.loginfo("State machine finished with outcome: %s", outcome)
    smach_viewer.stop()


if __name__ == "__main__":
    main()
