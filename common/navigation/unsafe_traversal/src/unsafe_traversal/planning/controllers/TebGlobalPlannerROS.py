from ..controller import PlannerController
from ...parameter_controller import ParameterOverrideController

TARGET_MIN_OBSTACLE_DIST = 0.02
TARGET_INFLATION_DIST = 0.2


class TebPlannerController(PlannerController):
    '''
    Parameters needed for TIAGo simulation.
    '''
    def __init__(self):
        self.planner_overrides = ParameterOverrideController(
            "/move_base/TebLocalPlannerROS")

    def set_unsafe(self, unsafe):
        if unsafe:
            self.planner_overrides.set_double_param(
                'inflation_dist', TARGET_INFLATION_DIST)
            self.planner_overrides.set_double_param(
                'min_obstacle_dist', TARGET_MIN_OBSTACLE_DIST)
        else:
            self.planner_overrides.restore_all()
