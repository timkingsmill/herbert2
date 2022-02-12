import odrive.utils

from odrive.utils import dump_errors

from odrive.enums import(
    AXIS_ERROR_NONE,
    MOTOR_ERROR_NONE,
    CONTROLLER_ERROR_NONE,
    ENCODER_ERROR_NONE
)    

# ---------------------------------------------------------------------------------

def check_error_state(axis) -> bool:
    if (axis.axis.error != AXIS_ERROR_NONE):
        return True

    if (axis.axis.motor.error != MOTOR_ERROR_NONE):
        return True

    if (axis.axis.controller.error != CONTROLLER_ERROR_NONE):
        return True

    if (axis.axis.encoder.error != ENCODER_ERROR_NONE):
        return True
    

    return False

# ---------------------------------------------------------------------------------
