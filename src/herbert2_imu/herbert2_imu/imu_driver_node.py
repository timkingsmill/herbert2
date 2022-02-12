from __future__ import print_function

from math import fabs
import serial
import serial.threaded
import threading
import time
import queue
import json

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from herbert2_geometry import Quaternion

# -------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------

class IMUReader(serial.threaded.LineReader):

    def __init__(self):
        super(IMUReader, self).__init__()

        self._connected = False
        self._queue= queue.Queue()

        self._event_thread = threading.Thread(target=self._run_event)
        self._event_thread.daemon = False
        self._event_thread.name = 'imu-event'

    # Implement Protocol class functions for Threaded Serial
    def connection_made(self, transport):
        """
        Overridden method from the LineReader class
        """
        super(IMUReader, self).connection_made(transport)

        print("Connected to the IMU.")
        
        # our adapter enables the module with RTS=low
        self.transport.serial.rts = False
        time.sleep(0.3)
        self.transport.serial.reset_input_buffer()

        self._connected = True
        # Start reading IMU data.
        self._event_thread.start()

    # ----------------------------------------------------------------

    def connection_lost(self, exc):
        """
        Overridden method from the LineReader class
        """
        super(IMUReader, self).connection_lost(exc)
        self._connected = False
        self._queue.queue.clear()
        print("Disconnected from the IMU.")

    # ----------------------------------------------------------------

    def handle_line(self, line):
        """
        Overridden method from the LineReader class
        Handle input from serial port.

        Keeps the depth of the queue short to 
        give the minimum processing delay for the
        most recent message data
        """
        if (self._connected):
            self._queue.put(line)
    
    # ----------------------------------------------------------------

    def handle_packet(self, packet):
        super(IMUReader, self).handle_packet(packet)

    # ----------------------------------------------------------------

    def _run_event(self):
        while (self._connected):
            try:
                if (not self._queue.empty()):
                    imu_data = self._queue.get()
                    if (self._parser_func):
                        # Parse the IMU data
                        self._parser_func(imu_data)
                    self._queue.task_done()    
            except Exception as e:
                print(str(e))

    # ----------------------------------------------------------------

    def set_parser(self, parser):
        self._parser_func = parser

    # ----------------------------------------------------------------


# ---------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------

class IMUDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('herbert2_imu_driver')
        
        self._imu_publisher = self.create_publisher(Imu, 'imu_data', qos_profile = qos_profile_sensor_data)

        self._serial = serial.Serial(port = '/dev/ttyACM0', baudrate = 115200, timeout = 10, rtscts = False)
        self._connectionThread = serial.threaded.ReaderThread(self._serial, IMUReader)
        self._connectionThread.start()
        self._connectionThread.protocol.set_parser(self.parse)

    def destroy_node(self) -> bool:
        self._connectionThread.close()
        super().destroy_node()
    
    def parse(self, line: str):
        if (line.startswith('log:')):
            self.get_logger().error(line)
        else:
            self.get_logger().info(line)
            try:
                quat = json.loads(line)
                qw = quat['quat_w']
                qx = quat['quat_x']
                qy = quat['quat_y']
                qz = quat['quat_z']

                quaternion: Quaternion = Quaternion(qw, qx, qy, qz)
                if (quaternion.is_valid()):
                    yaw = quaternion.get_yaw(True)

                    #self.get_logger().info(f'Yaw: {yaw:.6f}    Quat: {line}')
                    #time.sleep(0.1)

                    imu_message = Imu()
                    imu_message.header.stamp = Clock().now().to_msg()
                    imu_message.orientation.w = qw
                    imu_message.orientation.x = qx
                    imu_message.orientation.y = qy
                    imu_message.orientation.z = qz

                    # Publish and broadcast odometry.
                    self._imu_publisher.publish(imu_message)
                else:
                    self.get_logger().error('Invalid Quaternion')

            except Exception as exeption:
                self.get_logger().error(f'Failed to parse a message from the IMU device: {str(exeption)}')
                self.get_logger().error(line)

                
        
