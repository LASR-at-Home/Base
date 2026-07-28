from lasr_skills import GoToLocation, PlayMotion

import yasmin


class SafeGoToLocation(yasmin.StateMachine):
    def __init__(self, location_pose=None, location_param=None):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("location")
        self.add_input_key("motion_name")
        
        if location_param:
            location_param = location_param.upper()
            state_name = f"GO_TO_{location_param}"
        else:
            state_name = "SAFE_GO_TO_POINT"

        self.add_state(
            "PRE_NAV",
            PlayMotion("pre_navigation"),
            transitions={
                "succeeded": state_name,
                "aborted": state_name,
                "canceled": state_name,
            },
        )
        
        if location_param:
            location_param = location_param.lower()

        self.add_state(
            state_name,
            GoToLocation(location_param=location_param.lower()),
            transitions={"succeeded": "POST_NAV", "failed": state_name},
        )

        self.add_state(
            "POST_NAV",
            PlayMotion("post_navigation"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )
