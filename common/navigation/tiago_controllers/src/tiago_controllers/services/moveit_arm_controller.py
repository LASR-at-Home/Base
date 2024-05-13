#!/usr/bin/env python
from __future__ import print_function
import copy
from moveit_msgs.msg import DisplayTrajectory
import rospy
import moveit_commander
from moveit_commander import MoveItCommanderException
from std_srvs.srv import Empty

GRAB_OBJECT = [1.37, -1.32, -2.92, 2.25, -1.57, -0.85, 0.0]
INITIAL_POSITION = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]


class ArmTorsoController:
    def __init__(self):
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        _group_name = "arm_torso"
        self._group_names = self._robot.get_group_names()
        self._move_group = moveit_commander.MoveGroupCommander(name=_group_name, wait_for_servers=0)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                             DisplayTrajectory,
                                                             queue_size=20)

    def get_joint_values(self):
        return self._move_group.get_current_joint_values()

    def get_state(self):
        return self._robot.get_current_state()

    def stop(self):
        self._move_group.stop()

    def sync_reach_joint_space(self, torso_goal=None, arm_goals=None):
        """
        :param arm_goals: the position of the 7 arm joints
        :param torso_goal: the height of the torso
        """
        try:
            if arm_goals is None or len(arm_goals) != 7:
                rospy.logerr("you need to input the 7 arm goals")
                return None
            current_state = self.get_joint_values()
            joint_goal = self.__get_arm_goals(torso_goal, arm_goals)
            if joint_goal != current_state:
                result = self._move_group.go(joint_goal, wait=True)
                self.stop()
                return result
            rospy.loginfo("you're already at the position")
            return False
        except MoveItCommanderException as e:
            print("Service call failed: %s" % e)

    def __get_arm_goals(self, torso_goal, arm_goals):
        current_state = self.get_joint_values()
        joint_goal = copy.copy(current_state)
        if torso_goal is not None:
            joint_goal[0] = torso_goal

        for i in range(len(arm_goals)):
            if arm_goals[i]:
                joint_goal[i + 1] = arm_goals[i]
        return joint_goal

    def get_plan(self, torso_goal, arm_goals):
        plan = self._move_group.plan(self.__get_arm_goals(torso_goal, arm_goals))
        if not plan.joint_trajectory.joint_names:
            return None
        return plan

    def execute_plan(self, torso_goal, arm_goals):
        plan = self.get_plan(torso_goal, arm_goals)
        if not plan:
            return None
        return self._move_group.execute(plan, wait=True)

    def display_trajectory(self, plan):
        if plan:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self._robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self._display_trajectory_publisher.publish(display_trajectory)
        else:
            rospy.logwarn("There is no available plan!")

    def get_base_info(self):
        planning_frame = self._move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = self._move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        print("============ Available Planning Groups:", self._group_names)

        print("============ robot state:")
        print(self.get_joint_values())
        print("")

    def clear_octomap(self):
        rospy.wait_for_service('/clear_octomap')
        try:
            clear = rospy.ServiceProxy('/clear_octomap', Empty)
            clear()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    if rospy.get_published_topics(namespace='/move_group/display_planned_path'):
        a = ArmTorsoController()
        a.get_base_info()
        a.sync_reach_joint_space(0.1, INITIAL_POSITION)
        a.display_trajectory(a.get_plan(0.15, GRAB_OBJECT))
        a.clear_octomap()
        a.execute_plan(0.15, GRAB_OBJECT)
    else:
        rospy.loginfo('*'*50, "You are not running TIAGO", '*'*50)
