import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, Quaternion, Pose


# Object categories that belong in the dishwasher (dirty tableware + cutlery).
# In CATEGORY_MAP these all fall under "dish".
DISHWASHER_CATEGORIES = {"dish"}


class DecideDestination(yasmin.State):
    """
    Decides where the selected object must go, based on its category — this is
    the "task planning" core of the Pick and Place challenge:

        - tableware / cutlery (category "dish")   → dishwasher
        - the announced trash category            → trash bin
        - everything else                         → cabinet (then ChooseShelf)

    Manipulation is announce-only, so this state also loads the chosen
    destination's nav pose into blackboard["location"] for the generic
    GoToLocation state that follows.

    ROS 2 params:
        pick_and_place.trash_category      — category treated as trash ("" = none)
        pick_and_place.dishwasher.pose.*   — dishwasher nav pose
        pick_and_place.trash_bin.pose.*    — trash bin nav pose
        pick_and_place.cabinet.pose.*      — cabinet nav pose

    Blackboard inputs:
        object_category      : str
        selected_object_name : str

    Blackboard outputs:
        destination      : str   — "dishwasher" | "trash_bin" | "cabinet"
        destination_str  : str   — human phrase e.g. "the dishwasher"
        location         : Pose  — nav goal for GoToLocation
        chosen_shelf     : str   — cleared ("") for non-cabinet destinations
        chosen_shelf_str : str   — cleared ("") for non-cabinet destinations

    Outcomes:
        cabinet : object goes to the cabinet → run ChooseShelf next
        other   : object goes to dishwasher/trash → skip ChooseShelf
    """

    DEST_STR = {
        "dishwasher": "the dishwasher",
        "trash_bin":  "the trash bin",
        "cabinet":    "the cabinet",
    }

    def __init__(self):
        super().__init__(outcomes=["cabinet", "other"])
        self.add_input_key("object_category")
        self.add_input_key("selected_object_name")
        self.add_output_key("destination")
        self.add_output_key("destination_str")
        self.add_output_key("location")
        self.add_output_key("chosen_shelf")
        self.add_output_key("chosen_shelf_str")

        self.node = yasmin_ros.logger_node

    def execute(self, blackboard) -> str:
        name     = blackboard["selected_object_name"]
        category = (blackboard["object_category"] or "").lower()

        trash_category = self._get_str_param("pick_and_place.trash_category", "")

        # ── Route ─────────────────────────────────────────────────────────────
        if category in DISHWASHER_CATEGORIES:
            destination = "dishwasher"
        elif trash_category and category == trash_category.lower():
            destination = "trash_bin"
        else:
            destination = "cabinet"

        yasmin.YASMIN_LOG_INFO(
            f"'{name}' (category '{category}') -> {destination}."
        )

        # ── Load destination pose into blackboard["location"] ─────────────────
        blackboard["location"]        = self._load_pose(
            f"pick_and_place.{destination}.pose"
        )
        blackboard["destination"]     = destination
        blackboard["destination_str"] = self.DEST_STR.get(destination, destination)

        if destination == "cabinet":
            # ChooseShelf fills in chosen_shelf / chosen_shelf_str
            return "cabinet"

        # Non-cabinet: clear any stale shelf hint so InstructPlace omits it
        blackboard["chosen_shelf"]     = ""
        blackboard["chosen_shelf_str"] = ""
        return "other"

    # ── helpers ─────────────────────────────────────────────────────────────

    def _get_str_param(self, name: str, default: str) -> str:
        try:
            val = self.node.get_parameter(name).get_parameter_value().string_value
            return val if val else default
        except Exception:
            return default

    def _load_pose(self, prefix: str) -> Pose:
        """Reads <prefix>.position.* / .orientation.* params into a Pose.

        Never raises: missing components default to 0.0, and a fully-zero
        (invalid) quaternion is repaired to identity so navigation does not
        crash if a destination pose has not been configured yet.
        """
        def g(comp: str) -> float:
            p = f"{prefix}.{comp}"
            try:
                if not self.node.has_parameter(p):
                    self.node.declare_parameter(p, 0.0)
                return float(self.node.get_parameter(p).value)
            except Exception:
                return 0.0

        pose = Pose(
            position=Point(x=g("position.x"), y=g("position.y"), z=g("position.z")),
            orientation=Quaternion(
                x=g("orientation.x"), y=g("orientation.y"),
                z=g("orientation.z"), w=g("orientation.w"),
            ),
        )

        if (pose.orientation.x == 0.0 and pose.orientation.y == 0.0
                and pose.orientation.z == 0.0 and pose.orientation.w == 0.0):
            yasmin.YASMIN_LOG_WARN(
                f"Pose '{prefix}' looks unset (zero quaternion) — "
                f"check config.yaml. Using identity orientation."
            )
            pose.orientation.w = 1.0

        return pose