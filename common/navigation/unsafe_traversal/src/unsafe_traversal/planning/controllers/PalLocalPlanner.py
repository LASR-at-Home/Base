from ..controller import PlannerController
from ...parameter_controller import ParameterOverrideController

TARGET_SECURITY_DIST = 0.015
TARGET_INFLATION_RADIUS = 0.08


class PalPlannerController(PlannerController):
    '''
    Parameters needed for physical TIAGo robot.
    '''

    def __init__(self):
        self.planner_overrides = ParameterOverrideController(
            "/move_base/PalLocalPlanner")
        self.global_costmap_overrides = ParameterOverrideController(
            "/move_base/global_costmap/inflation_layer")

    def set_unsafe(self, unsafe):
        if unsafe:
            self.planner_overrides.set_double_param(
                'security_dist', TARGET_SECURITY_DIST)
            self.global_costmap_overrides.set_double_param(
                'inflation_radius', TARGET_INFLATION_RADIUS)
        else:
            self.planner_overrides.restore_all()
            self.global_costmap_overrides.restore_all()
