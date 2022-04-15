import time
from rclpy.node import Node

from .herbert2_robot import MotorController

from odrive.enums import(
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    ODRIVE_ERROR_NONE,
    INPUT_MODE_INACTIVE,      # 0
    INPUT_MODE_PASSTHROUGH,   # 1
    INPUT_MODE_TRAP_TRAJ,
)

# ....................................................................

class VelocityController(MotorController):

    def __init__(self, node: Node, motor_driver) -> None:
        super().__init__(node, motor_driver)
        self.get_logger().info('VelocityController')

    # .............................................................

    def do_initialize_axis(self, axis):
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # Wait for the axis to enter closed loop mode.
        #while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        #    time.sleep(0.1)

        # Set axis controllers to position control mode
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH 

    # .............................................................

    def do_initialize_odrive(self, odrive):
        pass

# ....................................................................
