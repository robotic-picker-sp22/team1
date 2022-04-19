from moveit_msgs.msg import MoveItErrorCodes


def moveit_error_string(val):
    """Returnss a string associated with a MoveItErrorCode.

    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg

    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """
    return {MoveItErrorCodes.SUCCESS: 'SUCCESS',
            MoveItErrorCodes.FAILURE: 'FAILURE',
            MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
            MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
            MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
            MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
            MoveItErrorCodes.PREEMPTED: 'PREEMPTED',
            MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
            MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
            MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
            MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
            MoveItErrorCodes.INVALID_LINK_NAME: 'INVALID_LINK_NAME',
            MoveItErrorCodes.INVALID_OBJECT_NAME: 'INVALID_OBJECT_NAME',
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'FRAME_TRANSFORM_FAILURE',
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'COLLISION_CHECKING_UNAVAILABLE',
            MoveItErrorCodes.ROBOT_STATE_STALE: 'ROBOT_STATE_STALE',
            MoveItErrorCodes.SENSOR_INFO_STALE: 'SENSOR_INFO_STALE',
            MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION'}.get(val,
            'UNKNOWN_ERROR_CODE')
