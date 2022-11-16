#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client
from keyboard_controller.models.kurf_run_settings import Speed

MAX_CHANGE = 3

FAST = 0.0, 0.7
NORMAL = 0.0, 0.5
SLOW = 0.1, 0.3

class SpeedConfiguration():
    def __init__(self, speed_mode):
        self.client = dynamic_reconfigure.client.Client("/move_base/PalLocalPlanner", timeout=30,
                                                        config_callback=self.callback())

        if speed_mode == Speed.FAST:
            self.change_robot_speed("min_vel_x", FAST[0])
            self.change_robot_speed("max_vel_x", FAST[1])
        elif speed_mode == Speed.NORMAL:
            self.change_robot_speed("min_vel_x", NORMAL[0])
            self.change_robot_speed("max_vel_x", NORMAL[1])
        elif speed_mode == Speed.SLOW:
            self.change_robot_speed("min_vel_x", SLOW[0])
            self.change_robot_speed("max_vel_x", SLOW[1])

    def callback(config):
        pass

    def is_within_tol(self, value, default=0.0):
        return abs(value) <= abs(default * MAX_CHANGE) if default != 0.0 else abs(value) <= abs(default + MAX_CHANGE)

    def change_robot_speed(self, param, value, tol=0.1):
        try:
            rospy.logwarn(f"Config before change of {param} set to {self.client.get_configuration()[param]}")
            print("   ")
            min_param = [x for x in self.client.get_parameter_descriptions() if x['name'].lower() == param.lower()][
                0].get('min')
            max_param = [x for x in self.client.get_parameter_descriptions() if x['name'].lower() == param.lower()][
                0].get('max')
            type_param = [x for x in self.client.get_parameter_descriptions() if x['name'].lower() == param.lower()][
                0].get('type')
            default = [x for x in self.client.get_parameter_descriptions() if x['name'].lower() == param.lower()][
                0].get('default')
            # rospy.loginfo(f"check bounds of min_param= {min_param}, max_param = {max_param}, default_value= {default}, type_para, = {type_param} ...")
            if not max_param > value > min_param or not self.is_within_tol(value, default):#or type(value) != type_param
                rospy.logwarn(f" Out of bounds value and setting it to default")
                self.client.update_configuration({param: default})
            else:
                rospy.loginfo(f"changing the value of {param} with {value}")
                self.client.update_configuration({param: value})
        except:
            rospy.logwarn(f"cannot change the value of {param} with {value}")



if __name__ == '__main__':
    rospy.init_node("speed_configuration", anonymous=True)
    _speed = SpeedConfiguration(Speed.NORMAL)