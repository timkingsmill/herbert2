"""
Herbert2 Robot Module.

Implements the Herbert2Robot class.


"""
import threading
from time import sleep

from rclpy.node import Node

from .odrive_driver import ODriveDriver
#from .diff_odrive_controller import DiffODriveController
from .sensors import JointSensors 

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

        # Create the ODrive Driver.
        self._odrive_driver = ODriveDriver(self)
        
        # Calibrate the driver
        self._odrive_driver.calibrate()
        self._odrive_driver.enter_velocity_control()

        # Init odometry
        #self._odrive_controller = DiffODriveController(self)

        self._add_sensors()

        # Setup thread to update all sensor data.
        self._update_thread = threading.Thread(target = self._update_thread_proc)
        self._update_thread.daemon = False
        self._update_thread.name = 'update-thread'

        self._connected = True
        self._update_thread.start()

    # -----------------------------------------------------------
    
    def destroy_node(self):
        self._connected = False
        self._odrive_driver._stop()

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

    """
    def _update_timer_callback(self) -> None:
        if (self._initialized):
            for sensor in self._sensors:
                sensor.update()
    """

    # -----------------------------------------------------------

    def _update_thread_proc(self) -> None:
        while (self._connected):
            for sensor in self._sensors:
                sensor.update()
            sleep(0.005)


# ---------------------------------------------------------------
# ---------------------------------------------------------------
