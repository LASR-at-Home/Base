import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode
import rclpy
import time
# NOTE: lasr_llm_interfaces is imported LAZILY inside _classify_with_llm so that a
# broken/stale typesupport (.so) never crashes state-machine construction. The LLM
# is only a 3rd-tier fallback after param lookup + CATEGORY_MAP.



CATEGORY_MAP = {
    "fruit": {
        "apple", "banana", "orange",
    },
    "vegetable": {
        "carrot", "tomato", "cucumber", "lettuce", "onion", "broccoli",
        "cabbage", "pepper", "zucchini", "radish", "corn", "potato", "garlic",
    },
    "drink": {
        "bottle", "water bottle", "juice", "milk",
        "soda can", "coffee cup", "energy drink", "thermos", "coke", "red bull", "iced tea",
    },
    "snack": {
        "chips", "crackers", "candy", "chocolate bar", "cookie",
        "snack bag", "biscuit", "granola bar", "popcorn", "pringles", "crisps",
    },
    "cleaning": {
        "soap", "sponge", "brush", "cleaner", "detergent", "tissue box",
        "toilet paper", "broom", "mop", "spray bottle", "bucket", "toothpaste",
    },
    "cereal": {
        "cereal", "cereal box", "oats", "muesli",
    },
    "dish": {
        "fork", "knife", "spoon", "plate", "bowl", "wine glass",
        "mug", "chopsticks", "cup",
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
        3. LLM fallback via /storing_groceries/query_llm

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

        assert task in ("object", "shelf"), \
            f"ClassifyCategory task must be 'object' or 'shelf', got '{task}'"

        self._task = task

        if task == "object":
            self.add_input_key("object_name")
            self.add_output_key("object_category")
        else:
            self.add_input_key("object_names")
            self.add_output_key("shelf_category")

        self.node = yasmin_ros.logger_node
        # Created lazily on first LLM use (see _classify_with_llm).
        self._llm_client = None
        self._llm_srv_type = None

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
                self.node.get_parameter(
                    f"pick_and_place.objects.{name}.category"
                )
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
        # Lazily import the interface + create the client on first use only.
        # BOTH the import AND create_client are wrapped: the typesupport error
        # for a broken lasr_llm_interfaces fires at create_client, so it must be
        # inside the try. On any failure we skip the LLM tier instead of crashing.
        if self._llm_client is None:
            try:
                from lasr_llm_interfaces.srv import StoringGroceriesQueryLlm
                self._llm_srv_type = StoringGroceriesQueryLlm
                self._llm_client = self.node.create_client(
                    StoringGroceriesQueryLlm, "/storing_groceries/query_llm"
                )
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(
                    f"lasr_llm_interfaces unavailable — skipping LLM tier ({e})."
                )
                return None

        if not self._llm_client.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("LLM service not available — skipping LLM tier.")
            return None

        req = self._llm_srv_type.Request()
        req.llm_input = [name]
        req.task = "ClassifyObject"

        future = self._llm_client.call_async(req)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        response = future.result() if future.done() else None

        if response is None:
            yasmin.YASMIN_LOG_WARN("LLM call failed/timed out.")
            return None

        category = response.category.strip().lower()
        return category or None