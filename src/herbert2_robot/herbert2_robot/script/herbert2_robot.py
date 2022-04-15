"""
Herbert2 Robot Module.

Implements the Herbert2Robot class.


"""
import threading
from time import sleep

from rclpy.node import Node

from .motor_driver import MotorDriver
from .sensors import JointSensors 

from .motor_controller import MotorController
#from .trajectory_controller import TrajectoryController
from .trajectory_planer import TrajectoryPlaner

from .velocity_controller import VelocityController

# IMUSensor
# from ros2.ros2_lidar_driver_py.ros2_lidar_driver_py import LidarDriver

# ---------------------------------------------------------------
# ---------------------------------------------------------------

class Herbert2Robot(Node):
    """
    Herbert2 Robot Node.
    """

    # -----------------------------------------------------------

    def __init__(self) -> None:
        """ Herbert2 Robot Constructor """
        super().__init__(node_name = 'herbert2_robot')

        # Setup the motor controller and driver.        
        self._motor_controller: MotorController = VelocityController(self, MotorDriver(self))
        self._motor_controller.initialize()
        # Calibrate the motor driver
        self._motor_controller.calibrate()

        # Create the trajectory planer.
        self._trajectory_planner: TrajectoryPlaner = TrajectoryPlaner(self._motor_controller)

        #self._motor_controller.set_input_velocity(2.0, 6.0, 3.0)


        #self._add_sensors()

        # Setup thread to update all sensor data.
        #self._update_thread = threading.Thread(target = self._update_thread_proc)
        #self._update_thread.daemon = False
        #self._update_thread.name = 'update-thread'

        #self._connected = True
        #self._update_thread.start()

    # -----------------------------------------------------------
    
    def destroy_node(self):
        #self._connected = False
        self._motor_controller.stop()

        super().destroy_node()
        self.get_logger().info('Destroyed the Herbert2 robot node')

    # -----------------------------------------------------------
    # -----------------------------------------------------------
    
    def _add_sensors(self):
        self._sensors = [
            # All 3 motor positions are published by JointSensors
            JointSensors(self, self._odrive_driver)
            #IMUSensor(self),
        ]
    
    # -----------------------------------------------------------

    def _update_thread_proc(self) -> None:
        while (self._connected):
            for sensor in self._sensors:
                sensor.update()
            sleep(0.005)


# ---------------------------------------------------------------
# ---------------------------------------------------------------
