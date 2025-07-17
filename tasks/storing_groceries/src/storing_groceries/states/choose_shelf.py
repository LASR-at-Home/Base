import rospy

import smach


class ChooseShelf(smach.State):

    def __init__(self, use_arm: bool = True):
        super().__init__(
            outcomes=["succeeded", "failed"],
            output_keys=["chosen_shelf", "shelf_data", "chosen_shelf_str"],
            input_keys=["selected_object_name", "shelf_data"],
        )
        self._use_arm = use_arm

    def execute(self, userdata):
        object_category = rospy.get_param(
            f"/storing_groceries/objects/{userdata.selected_object_name}/category"
        )
        object_name = userdata.selected_object_name

        rospy.loginfo(f"Placing object '{object_name}' of category: {object_category}")
        rospy.logwarn(userdata.shelf_data)

        chosen_shelf = None
        chosen_shelf_str = ""  # Default to empty

        max_count = -1
        fallback_shelf = None
        min_total_objects = float("inf")

        for shelf_name, shelf_info in userdata.shelf_data.items():
            # Exact match: dominant category
            if shelf_info["category"] == object_category:
                chosen_shelf = shelf_name
                rospy.loginfo(f"Found dominant category shelf: {chosen_shelf}")
                break

            # Track shelf with most of the same category
            count = shelf_info["category_counts"].get(object_category, 0)
            if count > max_count:
                max_count = count
                chosen_shelf = shelf_name
                rospy.loginfo(
                    f"Found shelf with {count} matching category items: {shelf_name}"
                )

            # Track fallback: least full shelf
            total_objects = len(shelf_info.get("objects", []))
            if total_objects < min_total_objects:
                min_total_objects = total_objects
                fallback_shelf = shelf_name

        # Try to find an empty shelf and set its category
        if chosen_shelf is None or max_count == 0:
            for shelf_name, shelf_info in userdata.shelf_data.items():
                if shelf_info["category"] == "empty":
                    chosen_shelf = shelf_name
                    userdata.shelf_data[shelf_name]["category"] = object_category
                    rospy.loginfo(f"Using empty shelf: {chosen_shelf}")
                    break

        # Final fallback
        if not chosen_shelf and fallback_shelf:
            chosen_shelf = fallback_shelf
            rospy.logwarn(
                f"No category matches or empty shelves. Fallback to: {chosen_shelf}"
            )

        if chosen_shelf:
            shelf_info = userdata.shelf_data[chosen_shelf]

            # Determine state BEFORE update
            was_empty = shelf_info["category"] == "empty"
            category_previously_present = object_category in shelf_info.get(
                "category_counts", {}
            )

            # Update object list
            if "objects" not in shelf_info:
                shelf_info["objects"] = []
            shelf_info["objects"].append(object_name)

            # Update category count
            if "category_counts" not in shelf_info:
                shelf_info["category_counts"] = {}
            shelf_info["category_counts"][object_category] = (
                shelf_info["category_counts"].get(object_category, 0) + 1
            )

            # Re-evaluate dominant category
            new_dominant_category = max(
                shelf_info["category_counts"].items(), key=lambda x: x[1]
            )[0]
            shelf_info["category"] = new_dominant_category

            # Set chosen_shelf_str only if category was already on shelf
            if not was_empty and category_previously_present:
                chosen_shelf_str = f"near the {object_category}"
            else:
                chosen_shelf_str = ""

            # Set outputs
            userdata.chosen_shelf = chosen_shelf
            userdata.chosen_shelf_str = chosen_shelf_str
            return "succeeded"

        else:
            rospy.logerr("No suitable shelf found.")
            return "failed"
