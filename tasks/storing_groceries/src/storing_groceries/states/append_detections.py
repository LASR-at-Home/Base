import rospy
import smach


class AppendDetections(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["cabinet_objects", "cabinets_objects"],
            output_keys=["cabinets_objects"],
        )

    def execute(self, userdata):
        rospy.loginfo(
            "[AppendDetectionsToCabinets] Appending detections to cabinets_objects."
        )

        if (
            not hasattr(userdata, "cabinets_objects")
            or userdata.cabinets_objects is None
        ):
            userdata.cabinets_objects = ["empty"]

        # Ensure the detections is a list
        current_detections = (
            userdata.cabinet_objects
            if isinstance(userdata.cabinet_objects, list)
            else []
        )

        rospy.loginfo(
            f"[AppendDetectionsToCabinets] Current detections: {current_detections}"
        )
        userdata.cabinets_objects.append(current_detections)
        rospy.loginfo(
            f"[AppendDetectionsToCabinets] Updated cabinets_objects: {userdata.cabinets_objects}"
        )
        return "succeeded"
