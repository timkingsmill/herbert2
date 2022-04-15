from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
import time
import weakref
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

from .motor_driver import MotorDriver

# ---------------------------------------------------------------------

class MotorController(ABC):

    # ---------------------------------------------------------------------
    # ---------------------------------------------------------------------

    def __init__(self, node: Node, motor_driver: MotorDriver) -> None:
        self._node_ref = weakref.ref(node)
        self._is_calibrated: bool = False
        self._is_initialized: bool = False
        self._motor_driver = motor_driver

    # ---------------------------------------------------------------------
    # ---------------------------------------------------------------------

    @property
    def motor_driver(self) -> MotorDriver:
        return self._motor_driver

    # ---------------------------------------------------------------------

    @property
    def node(self) -> Node:
        return self._node_ref()

    # ---------------------------------------------------------------------

    def initialize(self) -> None:
        if (self._is_initialized == False):
            self.stop()
            for odrv in self._motor_driver.odrv0, self._motor_driver.odrv1:
                self._initialize_odrive(odrv.odrv) 
            for axis in self.motor_driver:
                self._initialize_axis(axis)
            self.get_logger().info('Motor Controller initialization complete.')
            self._is_initialized = True

    # ---------------------------------------------------------------------

    def stop(self) -> None:
        self.get_logger().info('Stopping all axes.')
        self.set_input_velocity(0, 0, 0)
        for axis in self.motor_driver:
            axis.requested_state = AXIS_STATE_IDLE
            while (axis.current_state != AXIS_STATE_IDLE):
                self.get_logger().info('Unexpected Axis State.')
                time.sleep(0.1)

    # ---------------------------------------------------------------------

    @abstractmethod
    def do_initialize_axis(self, axis):
        pass

    # ---------------------------------------------------------------------

    @abstractmethod
    def do_initialize_odrive(self, odrive):
        pass

    # ---------------------------------------------------------------------

    def get_logger(self):
        return self.node.get_logger()

    # ---------------------------------------------------------------------

    def _initialize_axis(self, axis):
        self.get_logger().info('Initializing Axis.')

        axis.controller.config.enable_overspeed_error = False 

        # Set acceleration (turns/sec^2)
        axis.controller.config.vel_ramp_rate = 4 
        axis.controller.config.inertia = 0 

        # Enable velocity limit and set maximum speed for velocity control mode.
        axis.controller.config.enable_vel_limit = True 
        axis.controller.config.vel_limit = 4.0              # default = 2
        axis.controller.config.vel_limit_tolerance = 4.0    # default = 1.2

        # Enable velocity current limit.
        axis.controller.config.enable_current_mode_vel_limit = True

        axis.motor.config.calibration_current = 25
        axis.motor.config.torque_constant = 0.01
        axis.motor.config.current_lim = 40

        # Reset the encoder to zero position
        axis.encoder.set_linear_count(0)
        
        # Set the axis velocity to 0
        axis.controller.input_vel = 0

        # call the overriden method
        self.do_initialize_axis(axis)

        self.get_logger().info('Initializing Axis Completed.')

    # ---------------------------------------------------------------------

    def _initialize_odrive(self, odrv):
        # Initialize the ODrive.
        self.get_logger().info('Initializing ODrive.')
        odrv.clear_errors()
        
        odrv.config.dc_max_negative_current = float('-inf')
        odrv.config.dc_max_positive_current = float('inf')

        brake_resistance = 2.2
        odrv.config.brake_resistance = brake_resistance
        odrv.config.enable_brake_resistor = True
        odrv.config.dc_bus_undervoltage_trip_level = 0.0

        self.do_initialize_odrive(odrv)

    # ---------------------------------------------------------------------

    def calibrate(self) -> bool:
        self.initialize()
        do_calibrate: bool = False
        if (self._is_calibrated == False):
            for axis in self._motor_driver:
                if axis.motor.is_calibrated == False:
                    do_calibrate = True
                    break
        if do_calibrate:
            self._calibrate_axes()
            self._is_calibrated == True
            return True

        return False

    # ---------------------------------------------------------------------
    
    def _calibrate_axes(self):
        """
            Calibrate all axes.
            This method blocks until calibration is complete.
        """
        # Thread procedure
        def calibrate_thread_proc(axis):
            self._calibrate_axis(axis)

        if (self._is_calibrated == False):
            with ThreadPoolExecutor() as executer:
                executer.map(calibrate_thread_proc, self._motor_driver)
            
    # ---------------------------------------------------------------------
    
    def _calibrate_axis(self, axis) -> None:
        if (axis.current_state != AXIS_STATE_IDLE):
            self.get_logger().info('Unexpected Axis State.')

        # cache the current state (probably idle)
        state_cache = axis.current_state
        # Perform the calibration sequence.
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # Wait for the calibration to complete.
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        # restore the previous state 
        axis.requested_state = state_cache

    # ---------------------------------------------------------------------

    def set_input_position(self, a, b, c) -> None:
        self._motor_driver.axis0.controller.input_pos = a
        self._motor_driver.axis1.controller.input_pos = b
        self._motor_driver.axis2.controller.input_pos = c
        
        #self._axis1.axis.controller.input_pos = b
        #self._axis2.axis.controller.input_pos = c  

    # ---------------------------------------------------------------------

    def set_input_velocity(self, a, b, c) -> None:
        self._motor_driver.axis0.controller.input_vel = a
        self._motor_driver.axis1.controller.input_vel = b
        self._motor_driver.axis2.controller.input_vel = c

    # ---------------------------------------------------------------------

