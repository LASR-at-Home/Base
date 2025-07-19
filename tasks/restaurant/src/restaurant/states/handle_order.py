import rospy
import smach
import string


class HandleOrder(smach.StateMachine):
    def __init__(self, last_resort):
        from restaurant.states import SpeechRecovery

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["customer_transcription", "order_str", "order"],
            output_keys=["order", "order_str"],
        )
        self._possible_items = rospy.get_param("/restaurant/priors/items")
        self._possible_items = [item.replace("_", " ") for item in self._possible_items]
        self._last_resort = last_resort
        with self:
            smach.StateMachine.add(
                "PARSE_ORDER",
                ParseOrder(self._possible_items),
                transitions={"succeeded": "succeeded", "failed": "RECOVER_SPEECH"},
            )
            smach.StateMachine.add(
                "RECOVER_SPEECH",
                SpeechRecovery(self._possible_items, self._last_resort),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


class ParseOrder(smach.State):
    def __init__(self, possible_items):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["customer_transcription", "order_str", "order"],
            output_keys=["order", "order_str"],
        )
        self._possible_items = possible_items

    def execute(self, userdata):
        transcription_items_list = []
        transcription = (
            userdata.customer_transcription.lower()
            .translate(str.maketrans("", "", string.punctuation))
            .strip()
        )

        for item in self._possible_items:
            if item in transcription:
                transcription_items_list.append(item)

        transcription_items_list = list(set(transcription_items_list))
        userdata.order = transcription_items_list
        userdata.order_str = self._construct_string(transcription_items_list)
        print(transcription)
        print(userdata.order, userdata.order_str)

        if len(transcription_items_list) < 2:
            return "failed"
        else:
            return "succeeded"

    def _construct_string(self, transcription_items_list):
        order_string = ""
        for item in transcription_items_list:
            order_string += f" one {item},"
        return order_string.strip(",")
