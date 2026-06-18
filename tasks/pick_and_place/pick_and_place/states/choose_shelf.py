import yasmin
import yasmin_ros


class ChooseShelf(yasmin.State):
    """
    Chooses the most appropriate shelf for the selected object based on
    the shelf_data built by ScanShelves.

    Expects object_category to already be in the blackboard — set by
    ClassifyCategory(task="object") which runs before this state.

    Selection priority:
        1. Shelf whose dominant category matches the object's category
        2. Shelf with the most items of the same category
        3. An empty shelf (assigns the object's category to it)
        4. Final fallback: the least full shelf

    Updates shelf_data in the blackboard after each placement so subsequent
    objects are placed correctly relative to what is already there.

    Blackboard inputs:
        object_category : str   — set by ClassifyCategory
        selected_object_name : str
        shelf_data : dict       — built by ScanShelves

    Blackboard outputs:
        chosen_shelf     : str  — shelf ID e.g. "shelf_1"
        chosen_shelf_str : str  — placement hint e.g. "near the cereal"
        shelf_data       : dict — updated with new placement
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("object_category")
        self.add_input_key("selected_object_name")
        self.add_input_key("shelf_data")
        self.add_output_key("chosen_shelf")
        self.add_output_key("chosen_shelf_str")
        self.add_output_key("shelf_data")

    def execute(self, blackboard) -> str:
        object_name = blackboard["selected_object_name"]
        object_category = blackboard["object_category"]
        shelf_data = blackboard["shelf_data"]

        yasmin.YASMIN_LOG_INFO(
            f"Choosing shelf for '{object_name}' (category: '{object_category}')."
        )
        yasmin.YASMIN_LOG_INFO(f"Current shelf data: {shelf_data}")

        chosen_shelf = None
        chosen_shelf_str = ""
        max_count = -1
        fallback_shelf = None
        min_total_objects = float("inf")

        # ── Pass 1: find best matching shelf ─────────────────────────────────
        for shelf_name, shelf_info in shelf_data.items():

            # Priority 1: dominant category is an exact match
            if shelf_info["category"] == object_category:
                chosen_shelf = shelf_name
                yasmin.YASMIN_LOG_INFO(
                    f"Exact dominant category match on '{shelf_name}'."
                )
                break

            # Priority 2: shelf with the most items of this category
            count = shelf_info.get("category_counts", {}).get(object_category, 0)
            if count > max_count:
                max_count = count
                chosen_shelf = shelf_name
                yasmin.YASMIN_LOG_INFO(
                    f"Best category count so far ({count}) on '{shelf_name}'."
                )

            # Track fallback: least full shelf
            total_objects = len(shelf_info.get("objects", []))
            if total_objects < min_total_objects:
                min_total_objects = total_objects
                fallback_shelf = shelf_name

        # ── Pass 2: try an empty shelf ────────────────────────────────────────
        if chosen_shelf is None or max_count == 0:
            for shelf_name, shelf_info in shelf_data.items():
                if shelf_info["category"] == "empty":
                    chosen_shelf = shelf_name
                    shelf_data[shelf_name]["category"] = object_category
                    yasmin.YASMIN_LOG_INFO(f"Using empty shelf: '{shelf_name}'.")
                    break

        # ── Pass 3: final fallback ────────────────────────────────────────────
        if not chosen_shelf and fallback_shelf:
            chosen_shelf = fallback_shelf
            yasmin.YASMIN_LOG_WARN(
                f"No category match or empty shelf. "
                f"Falling back to least full shelf: '{chosen_shelf}'."
            )

        # ── Update shelf_data and set outputs ─────────────────────────────────
        if chosen_shelf:
            shelf_info = shelf_data[chosen_shelf]

            was_empty = shelf_info["category"] == "empty"
            category_previously_present = object_category in shelf_info.get(
                "category_counts", {}
            )

            shelf_info.setdefault("objects", []).append(object_name)
            shelf_info.setdefault("category_counts", {})[object_category] = (
                shelf_info["category_counts"].get(object_category, 0) + 1
            )

            new_dominant = max(
                shelf_info["category_counts"].items(), key=lambda x: x[1]
            )[0]
            shelf_info["category"] = new_dominant

            if not was_empty and category_previously_present:
                chosen_shelf_str = f"near the {object_category}"
            else:
                chosen_shelf_str = ""

            blackboard["chosen_shelf"] = chosen_shelf
            blackboard["chosen_shelf_str"] = chosen_shelf_str
            blackboard["shelf_data"] = shelf_data

            yasmin.YASMIN_LOG_INFO(
                f"Chose shelf '{chosen_shelf}'. "
                f"Placement hint: '{chosen_shelf_str}'."
            )
            return "succeeded"

        yasmin.YASMIN_LOG_ERROR("No suitable shelf found.")
        return "failed"
