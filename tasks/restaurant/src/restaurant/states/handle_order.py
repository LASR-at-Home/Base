import rospy

import smach

from lasr_llm_msgs.srv import (
    Llm,
    LlmRequest,
)


class HandleOrder(smach.State):
    def __init__(self):

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["customer_transcription"],
            output_keys=["order", "order_str"],
        )

        self._possible_items = rospy.get_param("/restaurant/priors/items")

        self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
        self._llm.wait_for_service()

    def execute(self, userdata):
        transcription = userdata.customer_transcription.lower()

        request = LlmRequest()
        request.system_prompt = (
            "You are a robot acting as a waiter in a restaurant."
            "  You are tasked with identifying items placed in an order by a customer."
            f" The possible items are: {','.join(self._possible_items)}."
            " You will receive input such as 'please get me a coffee, a cola and a sandwich'."
            " You should output only the items from the order separated by commas, for instance: 'coffee,cola,sandwich'."
            " Do not output anything else. If you fail to process the order, output 'none'."
            " An order can contain a maximum of three items."
        )
        request.prompt = transcription
        response = self._llm(request).output.strip()
        if response == "none":
            return "failed"
        else:
            userdata.order = response.split(",")
            userdata.order_str = response
            return "succeeded"
