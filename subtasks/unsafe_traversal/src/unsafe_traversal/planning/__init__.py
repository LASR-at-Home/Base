import rosservice

from .controllers import TebPlannerController, PalPlannerController


def get_planner_controller():
    '''
    Get the appropriate planner controller.
    '''

    if '/move_base/TebLocalPlannerROS/set_parameters' in rosservice.get_service_list():
        # running in TIAGo simulator
        return TebPlannerController()
    elif '/move_base/PalLocalPlanner/set_parameters' in rosservice.get_service_list():
        # running on TIAGo
        return PalPlannerController()
    else:
        return None
