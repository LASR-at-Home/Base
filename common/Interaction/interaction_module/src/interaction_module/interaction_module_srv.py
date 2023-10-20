#!/usr/bin/env python3
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionResponse
from listen_module.srv import DialogListen, DialogListenRequest, DialogListenResponse


class InteractionModule:
    def __init__(self):
        self.dialog_srv = rospy.ServiceProxy("/dialogflow_server", DialogListen)

        self.interaction_module_srv = rospy.Service("/interaction_module", AudioAndTextInteraction, self.interaction_module_cb)

    def interaction_module_cb(self, request):
        rospy.loginfo("Received audio request")
        resp = AudioAndTextInteractionResponse()
        print(request.action, request.subaction, request.query_text)
        result = self.dialog_srv(request.action, request.subaction, request.query_text)
        if result:
            resp.status = result.status
            resp.result = result.result
            # possibly call to talk here
        return resp


if __name__ == '__main__':
    rospy.init_node("interaction_module")
    rospy.loginfo("Interaction Module is up and running")
    interaction_module = InteractionModule()
    rospy.spin()