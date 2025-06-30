import smach
import rospy

from lasr_skills import Say


def stringify_guest_data(guest_data: dict) -> str:
    guest_str = ""
    if guest_data["attributes"]["long_hair"]:
        guest_str += "They have long hair. "
    else:
        guest_str += "They have short hair. "

    t_shirt = (
        "short sleeve"
        if guest_data["attributes"]["short_sleeve_t_shirt"]
        else "long sleeve"
    )

    if guest_data["attributes"]["glasses"] and guest_data["attributes"]["hat"]:
        guest_str += f"They are wearing a {t_shirt} top, glasses and a hat. "
    elif guest_data["attributes"]["glasses"] and not guest_data["attributes"]["hat"]:
        guest_str += f"They are wearing a {t_shirt} t shirt and glasses and they are not wearing a hat. "
    elif not guest_data["attributes"]["glasses"] and guest_data["attributes"]["hat"]:
        guest_str += f"They wearing a {t_shirt} t shirt and hat and they are not wearing glasses. "
    elif (
        not guest_data["attributes"]["glasses"] and not guest_data["attributes"]["hat"]
    ):
        guest_str += f"They wearing a {t_shirt} t shirt and they are not wearing glasses or a hat. "
    return guest_str


class WelcomeGuest(smach.StateMachine):
    """Class to welcome guest 2 to the party at the door, where we
    tell guest 2 the interests they have in common with guest 1."""

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        """
        Rough plan:
        - Say Hello {name} and welcome them to the party.
        - Guest {name} is already here. You'll recognise them as
          {describe}. I'd thought you might like to know
          that you share a common interest in {interest} with them.
         
        
        """

        with self:
            smach.StateMachine.add(
                "SAY_WELCOME",
                self.GetWelcomeMessage(),
                transitions={
                    "succeeded": "SAY_WELCOME_MESSAGE",
                    "failed": "failed",
                },
                remapping={
                    "guest_data": "guest_data",
                    "common_interest": "common_interest",
                    "welcome_message": "welcome_message",
                },
            )
            smach.StateMachine.add(
                "SAY_WELCOME_MESSAGE",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "welcome_message"},
            )

    class GetWelcomeMessage(smach.State):
        """State to get the welcome message for the guest."""

        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "common_interest"],
                output_keys=["welcome_message"],
            )

        def execute(self, userdata):
            guest_1_name = userdata.guest_data["guest1"]["name"]
            guest_2_name = userdata.guest_data["guest2"]["name"]

            userdata.welcome_message = (
                f"Hello {guest_2_name}, welcome to the party! "
                f"{guest_1_name} is already here, you'll recognise them as {stringify_guest_data(userdata.guest_data['guest1'])}."
            )
            return "succeeded"
