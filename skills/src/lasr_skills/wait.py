import smach
import rospy


class Wait(smach.State):
    def __init__(self, wait_time: int):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._wait_time = wait_time

    def execute(self, userdata) -> str:
        try:
            print(f"Waiting for {self._wait_time} seconds.")
            rospy.sleep(self._wait_time)
            return "succeeded"
        except:
            print("Waiting failed")
            return "failed"
