#!/usr/bin/env python3
import rospy
# from typing import Set, Tuple
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter


class ParameterOverrideController:
    '''
    Manages overriden parameters with the ability to rollback afterwards.
    '''

    # proxy: rospy.ServiceProxy
    # original_values: Config
    # overwritten: Set[Tuple[str, str]]

    def __init__(self, namespace):
        self.proxy = rospy.ServiceProxy(
            namespace + '/set_parameters', Reconfigure)
        self.original_values = None
        self.overwritten = set()

    def get_config(self):
        '''
        Return the original set values.
        '''

        if self.original_values is None:
            self.original_values = self.proxy().config

        return self.original_values

    def get_double_param(self, key):  # -> float:
        '''
        Get the original double parameter value.
        '''

        for entry in self.get_config().doubles:
            if entry.name == key:
                return entry.value

        return None

    def set_double_param(self, key, value):
        '''
        Override some double parameter.
        '''

        self.get_double_param(key)
        self.overwritten.add((key, "double"))

        config = Config()
        config.doubles.append(DoubleParameter(
            name=key,
            value=value
        ))

        self.proxy(config)

    def restore_all(self):
        '''
        Restore all overriden config options.
        '''

        config = Config()

        for (key, value_type) in self.overwritten:
            if value_type == 'double':
                config.doubles.append(DoubleParameter(
                    name=key,
                    value=self.get_double_param(key)
                ))

        self.overwritten = set()
        self.proxy(config)
