from rclpy.node import Node
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

from .herbert2_robot import MotorController

class TrajectoryController(MotorController):
    
    # .............................................................

    def __init__(self, node: Node, motor_driver) -> None:
        super().__init__(node, motor_driver)
        print('TrajectoryController')

    # .............................................................

    def do_initialize_axis(self, axis):
        axis.trap_traj.config.vel_limit = 4.0
        axis.trap_traj.config.accel_limit = 1.0
        axis.trap_traj.config.decel_limit = 1.0

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # Set axis controllers to position control mode
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ 

    # .............................................................

    def do_initialize_odrive(self, odrive):
        pass

    # .............................................................
