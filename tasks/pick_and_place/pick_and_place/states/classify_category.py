import yasmin
import yasmin_ros
import rclpy

from lasr_llm_msgs.srv import Llm

# Hardcoded category map as fallback when params are unavailable.
# Mirrors the category_map from the ROS 1 ClassifyCategory.
# Ideally these live in your config yaml under pick_and_place.objects.<name>.category
CATEGORY_MAP = {
    "fruit": {
        "apple",
        "banana",
        "orange",
        "grape",
        "pineapple",
        "lemon",
        "lime",
        "peach",
        "plum",
        "pear",
        "mango",
        "watermelon",
        "strawberry",
        "blueberry",
    },
    "vegetable": {
        "carrot",
        "tomato",
        "cucumber",
        "lettuce",
        "onion",
        "broccoli",
        "cabbage",
        "pepper",
        "zucchini",
        "radish",
        "corn",
        "potato",
        "garlic",
    },
    "beverage": {
        "bottle",
        "can",
        "water bottle",
        "juice box",
        "milk carton",
        "soda can",
        "coffee cup",
        "energy drink",
        "thermos",
    },
    "snack": {
        "chips",
        "crackers",
        "candy",
        "chocolate bar",
        "cookie",
        "snack bag",
        "biscuit",
        "granola bar",
        "popcorn",
    },
    "cleaning": {
        "soap",
        "sponge",
        "brush",
        "cleaner",
        "detergent",
        "tissue box",
        "toilet paper",
        "broom",
        "mop",
        "spray bottle",
        "bucket",
    },
    "cereal": {
        "cereal",
        "cereal box",
        "oats",
        "muesli",
    },
    "dish": {
        "fork",
        "knife",
        "spoon",
        "plate",
        "bowl",
        "cup",
        "wine glass",
        "mug",
        "chopsticks",
    },
}


class ClassifyCategory(yasmin.State):
    """
    Classifies an object or a list of objects into a category.

    Used in two places in the pipeline:
        1. Inside ScanShelves — to determine the dominant category of each shelf
           from the list of objects detected on it.
        2. After SelectAndVisualiseObject — to classify the selected table object
           before ChooseShelf runs.

    Classification priority:
        1. ROS 2 param lookup  (pick_and_place.objects.<name>.category)
        2. Hardcoded CATEGORY_MAP
        3. LLM fallback via /lasr_llm/llm

    Blackboard inputs:
        object_name  : str        — single object name (used when task="object")
        object_names : List[str]  — list of names     (used when task="shelf")

    Blackboard outputs:
        object_category  : str   — category for a single object
        shelf_category   : str   — dominant category for a shelf
    """

    def __init__(self, task: str = "object"):
        """
        Args:
            task: "object" — classify a single object name from blackboard["object_name"]
                  "shelf"  — classify a shelf from blackboard["object_names"] (list)
        """
        super().__init__(outcomes=["succeeded", "failed", "empty"])

        assert task in (
            "object",
            "shelf",
        ), f"ClassifyCategory task must be 'object' or 'shelf', got '{task}'"

        self._task = task

        if task == "object":
            self.add_input_key("object_name")
            self.add_output_key("object_category")
        else:
            self.add_input_key("object_names")
            self.add_output_key("shelf_category")

        self.node = yasmin_ros.get_node()
        self._llm_client = self.node.create_client(Llm, "/lasr_llm/llm")

    def execute(self, blackboard) -> str:
        if self._task == "object":
            return self._classify_object(blackboard)
        else:
            return self._classify_shelf(blackboard)

    # ── Task handlers ─────────────────────────────────────────────────────────

    def _classify_object(self, blackboard) -> str:
        """Classifies a single object name into a category."""
        name = blackboard["object_name"]
        if not name:
            yasmin.YASMIN_LOG_WARN("object_name is empty.")
            return "empty"

        category = self._get_category(name.lower())
        if category:
            blackboard["object_category"] = category
            yasmin.YASMIN_LOG_INFO(f"Classified '{name}' as '{category}'.")
            return "succeeded"

        yasmin.YASMIN_LOG_WARN(f"Could not classify '{name}'.")
        return "failed"

    def _classify_shelf(self, blackboard) -> str:
        """
        Classifies a shelf by finding the dominant category across all
        object names detected on it.
        """
        names = blackboard["object_names"]
        if not names:
            blackboard["shelf_category"] = "empty"
            return "succeeded"

        from collections import Counter

        category_counts = Counter()

        for name in names:
            category = self._get_category(name.lower())
            if category:
                category_counts[category] += 1

        if category_counts:
            dominant = category_counts.most_common(1)[0][0]
            blackboard["shelf_category"] = dominant
            yasmin.YASMIN_LOG_INFO(
                f"Shelf dominant category: '{dominant}' "
                f"from counts {dict(category_counts)}."
            )
        else:
            blackboard["shelf_category"] = "unknown"
            yasmin.YASMIN_LOG_WARN("Could not classify any objects on shelf.")

        return "succeeded"

    # ── Classification helpers ────────────────────────────────────────────────

    def _get_category(self, name: str) -> str | None:
        """
        Returns the category for an object name using three fallback levels:
            1. ROS 2 param
            2. Hardcoded CATEGORY_MAP
            3. LLM
        """
        # 1. Param lookup
        try:
            category = (
                self.node.get_parameter(f"pick_and_place.objects.{name}.category")
                .get_parameter_value()
                .string_value
            )
            if category:
                return category
        except Exception:
            pass

        # 2. Hardcoded map
        for category, items in CATEGORY_MAP.items():
            if name in items:
                return category

        # 3. LLM fallback
        return self._classify_with_llm(name)

    def _classify_with_llm(self, name: str) -> str | None:
        """Calls the LLM service to determine the category of an unknown object."""
        if not self._llm_client.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("LLM service not available.")
            return None

        category_list = ", ".join(sorted(CATEGORY_MAP.keys()))

        req = Llm.Request()
        req.system_prompt = (
            "You are a robot classifying household objects into categories. "
            "Respond with only one word from the list provided."
        )
        req.prompt = (
            f"Which category does '{name}' belong to most? "
            f"Choose from: {category_list}."
        )
        req.max_tokens = 10

        future = self._llm_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response is None:
            yasmin.YASMIN_LOG_WARN("LLM call failed.")
            return None

        words = response.output.strip().lower().replace(",", "").split()
        for word in words:
            if word in CATEGORY_MAP:
                return word

        yasmin.YASMIN_LOG_WARN(
            f"LLM response '{response.output}' didn't match any known category."
        )
        return None
