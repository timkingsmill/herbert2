import fibre
import time
import weakref

from concurrent.futures import ThreadPoolExecutor
from typing import Iterator, List

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from herbert2_geometry.robot_maths import wheel_velocity_from_vector
import odrive
from odrive.utils import dump_errors

from odrive.enums import(
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,

    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,

    ODRIVE_ERROR_NONE,

    INPUT_MODE_INACTIVE,      # 0
    INPUT_MODE_PASSTHROUGH,   # 1
)

from .node_decorator import NodeDecorator

# ---------------------------------------------------------------
# ---------------------------------------------------------------

class ODriveChannel: pass
class ODriveAxis: pass

# ---------------------------------------------------------------
# ---------------------------------------------------------------

class ODriveDriver(NodeDecorator): 

    MAX_LINEAR_VELOCITY = 1.0

    # -----------------------------------------------------------

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        
        """ ODrive Driver constructor """
        self.get_logger().info('Initializing Herbert2 ODrive Driver Node') 
        """
        Connect to the ODrive controller board.
        """
        # Find a connected ODrive (this will block until connected)
        odrives = fibre.find_any(find_multiple=2)

        if (len(odrives) != 2):
            msg = 'Unable to connect to both ODrive Driver Channels'
            self.get_logger().error(msg) 
            raise Exception(msg) 

        for index in [0, 1]:
            drive_name: str = f'odrv{index}'

            # Declare parameters
            self._node_ref().declare_parameter(drive_name + '.path', 'serial:/dev/tty???')
            path: str = self._node_ref().get_parameter(drive_name + '.path').value

            self._node_ref().declare_parameter(drive_name + '.serial_number', 0)
            serial_num: int = self._node_ref().get_parameter(drive_name + '.serial_number').value

            # Check the serial numbers.
            if (odrives[index].serial_number == serial_num):
                self.get_logger().info(
                    f'ODrive Driver connected with the correct serial number. ({odrives[index].serial_number})')
            else:
                msg = f'Unable to assign an ODrive Driver. Serial number is not correct. {serial_num}'
                self.get_logger().fatal(msg) 
                raise Exception(msg) 


        self._odrv0 = ODriveChannel(node, self, odrives[0])
        self._config_channel(self._odrv0)

        self._odrv1 = ODriveChannel(node, self, odrives[1])
        self._config_channel(self._odrv1)
        
        self._config_axis(self._axis0)
        self._config_axis(self._axis1)
        self._config_axis(self._axis2)

        # Initialise subscribers
        
        self._velocity_sub = self._node_ref().create_subscription(Twist, 
                            'cmd_vel', 
                            self._velocity_message_callback, qos_profile = qos_profile_sensor_data)


        self._is_calibrated = False
        self.get_logger().info('The ODrive Driver is ready for calibration.') 

    # -----------------------------------------------------------

    def calibrate(self) -> bool:
        """
            Return true if calibration proceedure is executed
            Return false if calibration has already been executed.
        """
        if (self._is_calibrated == False):
            from .calibration_service import CalibrationService
            self._calibration_service = CalibrationService(self.node, self)
        
            calibrate: bool = False
            for axis in self._axes:
                if axis.axis.motor.is_calibrated == False:
                    calibrate = True
                    break

            # TESTING.   FORCES CALIBRATION
            calibrate = True    

            if calibrate:
                self._calibrate_axes()
                self._is_calibrated == True
                self.get_logger().info('The ODrive Driver is calibrated and ready to accept ROS commands and messages.') 
                return True 
        else:
            return False

    # -----------------------------------------------------------

    def enter_position_control(self):
        for axis in self._axes:
            axis.enter_position_control()

    # -----------------------------------------------------------

    def enter_velocity_control(self):
        for axis in self._axes:
            axis.enter_velocity_control()

    # -----------------------------------------------------------

    def get_motor_positions(self):
        """
          Invert the position estimates to swap
          clockwise and anti-clockwise rotation.           
        """
        return [
            -(self._axis0.axis.encoder.pos_estimate),
            -(self._axis1.axis.encoder.pos_estimate),
            -(self._axis2.axis.encoder.pos_estimate),
        ]

    # -----------------------------------------------------------

    def _calibrate_axes(self):
        """
        Calibrate all axes.
        This method blocks until calibration is complete.
        """
        if (self._is_calibrated == False):
            self._node_ref().get_logger().info('Calibrating all axis.') 

            def calibrate_axis(axis: ODriveAxis):
                axis.calibrate_axis()

            with ThreadPoolExecutor() as executer:
                executer.map(calibrate_axis, self._axes)

            self._node_ref().get_logger().info('Calibrating all axis complete') 

    # -----------------------------------------------------------

    def _check_error_status(self) -> bool:
        for odrive in [self._odrv0._odrive, self._odrv1._odrive]:
            if (odrive.error != ODRIVE_ERROR_NONE):  
                dump_errors(odrive)
                return True
        return False

    # -----------------------------------------------------------

    @property
    def _axis0(self) -> ODriveAxis:
        if (self._odrv0):
            return self._odrv0.axis0
        else:
            return None

    # -----------------------------------------------------------

    @property
    def _axis1(self) -> ODriveAxis:
        if (self._odrv0):
            return self._odrv0.axis1
        else:
            return None

    # -----------------------------------------------------------

    @property
    def _axis2(self) -> ODriveAxis:
        if (self._odrv1):
            return self._odrv1.axis0
        else:
            return None

    # -----------------------------------------------------------

    @property
    def _axes(self) -> Iterator[ODriveAxis]:
        yield from [self._axis0, self._axis1, self._axis2]

    # -----------------------------------------------------------

    def _config_axis(self, axis: ODriveAxis) -> None:
        if (axis):
            axis.config_axis()

    # -----------------------------------------------------------

    def _config_channel(self, channel: ODriveChannel) -> None:
        channel.config_channel()

    # -----------------------------------------------------------

    def _velocity_message_callback(self, msg: Twist):
        """ 
        Handle velocity messages 
        """
        print('_velocity_message_callback()')

        # Rotate the robot.
        #angular_velocity: float = msg.angular.z 
        #setpoints = [1,1,1]   #[angular_velocity, angular_velocity, angular_velocity]

        #print(angular_velocity) 

        #self._set_velocity(setpoints)



        # Calc velocity for the axes
        #
        a, b, c = wheel_velocity_from_vector(msg.linear.x, msg.linear.y)
        self._set_velocity(
            [a * ODriveDriver.MAX_LINEAR_VELOCITY, 
             b * ODriveDriver.MAX_LINEAR_VELOCITY, 
             c * ODriveDriver.MAX_LINEAR_VELOCITY]
        )

    # -----------------------------------------------------------

    def _set_position(self, setpoints):
        # set velocity to 0.
        #self._set_velocity([0, 0, 0])

        self._axis0.axis.controller.input_pos = setpoints[0]
        self._axis1.axis.controller.input_pos = setpoints[1]
        self._axis2.axis.controller.input_pos = setpoints[2]

    # -----------------------------------------------------------

    def _set_velocity(self, setpoints) -> bool:
        try:
            if (self._check_error_status() == False):
                self._axis0.axis.controller.input_vel = setpoints[0]    # Motor A
                self._axis1.axis.controller.input_vel = setpoints[1]    # Motor B
                self._axis2.axis.controller.input_vel = setpoints[2]    # Motor C
                return True
            else:
                return False    
        except:
            self._node_ref().get_logger().error('Failed to set the velocity of the odrive axes')


    # -----------------------------------------------------------

    def _stop(self) -> None:
        """ Shutdown everything """
        self._set_velocity([0, 0, 0])
        for axis in self._axes:
            axis.stop()


    # -----------------------------------------------------------

# ---------------------------------------------------------------
# ---------------------------------------------------------------

class ODriveChannel():

    # -----------------------------------------------------------
    __id: int = -1
    """ instance id class variable """

    # -----------------------------------------------------------

    def __init__(self, node: Node, controller: ODriveDriver, odrive) -> None:
        """ ODriveChannel Constructor """
        self._node_ref = weakref.ref(node)
        self._node_ref().get_logger().info('Initializing Herbert2 ODrive Channel') 

        self._controller = controller
        self._odrive = odrive

        self._axis0 = ODriveAxis(node, self, odrive.axis0)
        self._axis1 = ODriveAxis(node, self, odrive.axis1)

        ODriveChannel.__id += 1
        self._odrive_name = format('odrv{0}'.format(ODriveChannel.__id))

        odrive.clear_errors()


    # ----------------------------------------------------------

    @property
    def axis0(self) -> ODriveAxis:
        return self._axis0

    # ----------------------------------------------------------

    @property
    def axis1(self) -> ODriveAxis:
        return self._axis1

    # ----------------------------------------------------------

    def config_channel(self) -> None:
        if (self._odrive):
            self._odrive.clear_errors()    

            self._odrive.config.dc_max_negative_current = float('-inf')
            self._odrive.config.dc_max_positive_current = float('inf')

            brake_resistance = 2.2
            self._odrive.config.brake_resistance = brake_resistance
            self._odrive.config.enable_brake_resistor = True

            self._odrive.config.dc_bus_undervoltage_trip_level = 0.0

    # ----------------------------------------------------------

# ---------------------------------------------------------------
# ---------------------------------------------------------------

class ODriveAxis(): 

    # -----------------------------------------------------------

    __id: int = -1
    """ instance id class variable """

    # -----------------------------------------------------------

    def __init__(self, node: Node, channel: ODriveChannel, axis) -> None:
        """ ODrive constructor """
        self._node_ref = weakref.ref(node)
        self._channel = channel
        self._odrive = channel._odrive 

        ODriveAxis.__id += 1
        self.__axis_id = ODriveAxis.__id
        self._axis_name = format('axis{0}'.format(ODriveAxis.__id))

        self._node_ref().get_logger().info(f'{self._axis_name} initialized. No errors detected.')

    # -----------------------------------------------------------

    @property
    def axis(self):
        lookup = {
            0: self._odrive.axis0,
            1: self._odrive.axis1,
            2: self._odrive.axis0,
            3: self._odrive.axis1,
        }
        return (lookup.get(self.__axis_id))

    # -----------------------------------------------------------
    
    def calibrate_axis(self) -> None:
        self._node_ref().get_logger().info(f'Calibrating {self._axis_name}.')
        try:

            if (self.axis.current_state != AXIS_STATE_IDLE):
                print('Unexpected Axis State.')

            # cache the current state (probably idle)
            state = self.axis.current_state

            #print(f'State: {state}')

            # Perform the calibration sequence.
            self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            while self.axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

            # restore the requested state 
            self.axis.requested_state = state
            self._node_ref().get_logger().info(f'Calibrated {self._axis_name} OK.')

        except Exception:        
            self._node_ref().get_logger().fatal(str(Exception))
            raise Exception('Calibrate Axis Raised Exception')

    # -----------------------------------------------------------
    
    def clear_errors(self) -> None:
        self._odrive.clear_errors()

    # -----------------------------------------------------------

    def config_axis(self) -> None:
        self.clear_errors()

        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        # Set acceleration (turns/sec^2)
        self.axis.controller.config.vel_ramp_rate = 4 
        self.axis.controller.config.inertia = 0 
        
        self.axis.controller.config.enable_overspeed_error = False 

        # Enable velocity limit and set maximum speed for velocity control mode.
        self.axis.controller.config.enable_vel_limit = True 
        self.axis.controller.config.vel_limit = 4.0              # default = 2
        self.axis.controller.config.vel_limit_tolerance = 4.0    # default = 1.2

        # Enable velocity current limit.
        self.axis.controller.config.enable_current_mode_vel_limit = True

        self.axis.motor.config.calibration_current = 25
        self.axis.motor.config.torque_constant = 0.01
        self.axis.motor.config.current_lim = 40

        # Reset the encoder to zero position
        self.axis.encoder.set_linear_count(0)
        
        # Set the axis velocity to 0
        self.axis.controller.input_vel = 0

        self.axis.requested_state = AXIS_STATE_IDLE
        while (self.axis.current_state != AXIS_STATE_IDLE):
            print('Unexpected Axis State.')
            time.sleep(0.1)


    # -----------------------------------------------------------

    def enter_position_control(self):
        self._node_ref().get_logger().info(f'Enter {self._axis_name} position control mode...')
        
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # Set axis controllers to position control mode
        self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH 

        self._node_ref().get_logger().info(f'Entered {self._axis_name} position control mode. No errors detected')

    # -----------------------------------------------------------

    def enter_velocity_control(self):
        self._node_ref().get_logger().info(f'Enter {self._axis_name} velocity control mode...')

        while self.axis.current_state != AXIS_STATE_IDLE:
            self._node_ref().get_logger().info(f'{self._axis_name} waiting for idle state.')
            time.sleep(0.1)

        # enter closed loop control (use the encoder)
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        while self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self._node_ref().get_logger().info(f'{self._axis_name} waiting to enter closed loop control mode.')
            time.sleep(0.1)

        # Set axis controllers to velocity control mode
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH 
        
        dump_errors(self._odrive)

        self._node_ref().get_logger().info(f'Entered {self._axis_name} velocity control mode. No errors detected')

    # -----------------------------------------------------------

    def stop(self):
        self.axis.requested_state = AXIS_STATE_IDLE


# ---------------------------------------------------------------
# ---------------------------------------------------------------
