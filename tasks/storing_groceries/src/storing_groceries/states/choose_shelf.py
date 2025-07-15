import rospy

import smach


class ChooseShelf(smach.State):

    def __init__(self, use_arm: bool = True):
        super().__init__(
            outcomes=["succeeded", "failed"],
            output_keys=["chosen_shelf", "shelf_data"],
            input_keys=["selected_object_name", "shelf_data"],
        )
        self._use_arm = use_arm

    def execute(self, userdata):

        object_category = rospy.get_param(
            f"/storing_groceries/objects/{userdata.selected_object_name}/category"
        )

        chosen_shelf = None
        rospy.logwarn(userdata.shelf_data)
        for shelf_name in userdata.shelf_data.keys():
            rospy.loginfo(userdata.shelf_data[shelf_name])
            if userdata.shelf_data[shelf_name]["category"] == object_category:
                chosen_shelf = shelf_name
                break
        if not chosen_shelf:
            for shelf_name in userdata.shelf_data.keys():
                if userdata.shelf_data[shelf_name]["category"] == "empty":
                    chosen_shelf = shelf_name
                    userdata.shelf_data[shelf_name]["category"] = object_category
                    break
        if chosen_shelf:
            userdata.chosen_shelf = chosen_shelf
            return "succeeded"
        else:
            return "failed"
