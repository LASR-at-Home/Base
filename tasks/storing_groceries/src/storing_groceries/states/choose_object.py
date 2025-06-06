import smach
import rospy

class ChooseObject(smach.State):
    def __init__(self, category_filter=None):
        """
        :param category_filter: string name of the category to include (e.g., 'cereal').
                                If None, all non-cereal objects are considered.
        """
        super().__init__(
            outcomes=["succeeded", "failed", "empty"],
            input_keys=["table_objects", "not_graspable"],
            output_keys=["table_object"],
        )
        self.category_filter = category_filter

        # Define known category sets
        self.category_map = {
            "cereal": {"cereal", "cereal_container"},
            # Add more categories if needed
        }

    def execute(self, userdata):
        if not userdata.table_objects:
            rospy.loginfo("No objects detected.")
            return "empty"

        category_set = self.category_map.get(self.category_filter, None)

        best_obj = None
        best_conf = -1.0

        for obj in userdata.table_objects:
            name = obj.get("name", "")
            conf = obj.get("confidence")
            bbox = obj.get("bbox")

            if name == "" or conf is None or bbox is None:
                continue

            skip = False

            for ng in userdata.not_graspable:
                if obj == ng:
                    skip = True
                    break
            if skip:
                continue

            # Category filtering logic
            if category_set:
                if name not in category_set:
                    continue  # Ignore if not in the desired category
            else:
                # Default mode: exclude cereal types
                if name in self.category_map["cereal"]:
                    
                    continue

            rospy.loginfo(f"Candidate: {name} (conf: {conf:.2f})")

            if conf > best_conf:
                best_obj = obj
                best_conf = conf

        if best_obj:
            userdata.table_object = best_obj
            rospy.loginfo(f"Selected: {best_obj['name']} (conf: {best_obj['confidence']:.2f})")
            return "succeeded"
        else:
            rospy.loginfo("No suitable object found for this category.")
            return "empty"
