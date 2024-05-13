#!/usr/bin/env python
import rospy
from tiago_controllers.services import ArmTorsoController
from tiago_controllers.srv import ArmTorsoPos, ArmTorsoPosResponse, ArmTorsoPosRequest


class ArmTorsoControllerSrv:
    def __init__(self):
        self._arm_torso_controller = ArmTorsoController()

    def __call__(self, req):
        if not isinstance(req, ArmTorsoPosRequest):
            raise rospy.ServiceException("Invalid request")
        else:
            if not req.plan:
                res = self._arm_torso_controller.sync_reach_joint_space(req.torso_goals, req.arm_goals)
            else:
                # self._arm_torso_controller.clear_octomap()
                res = self._arm_torso_controller.execute_plan(req.torso_goals, req.arm_goals)
            if res is None:
                raise rospy.ServiceException("Wrong request input")
            response = ArmTorsoPosResponse()
            response.result = res
        return response


if __name__ == "__main__":
    rospy.init_node("arm_torso_controller_server")
    if rospy.get_published_topics(namespace='/xtion'):
        server = ArmTorsoControllerSrv()
        service = rospy.Service('arm_torso_controller', ArmTorsoPos, server)
        rospy.loginfo("Arm Torso Controller Server initialised")
        rospy.spin()
    else:
        print('*'*50, "Arm Torso service NOT enables", '*'*50)
        rospy.spin()
