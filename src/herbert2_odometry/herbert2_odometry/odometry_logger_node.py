from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry as OdometryMessage

from herbert2_geometry import Quaternion

#...............................................................
#...............................................................

class OdometryLoggerNode(Node): 

    #...............................................................

    def __init__(self) -> None:
        """ Herbert2 Odometry Logger Node """
        super().__init__(node_name = 'herbert2_odometry_logger')

        # Subscribe to Joint State messages
        self._odometry_sub = self.create_subscription(
            OdometryMessage,
            '/odometry',
            self._odometry_subscriber_callback,
            qos_profile = qos_profile_sensor_data
        )
        self._odometry_received_timestamp = self.get_clock().now()

        # Initialise timers
        self._check_subscriber_timer = self.create_timer(
            1.0,  # unit: second
            self._check_subscriber_timer_callback)

    #...............................................................

    # ..............................................................
    # Check that the odometry message subscriber is receiving messages.
    def _check_subscriber_timer_callback(self):
        # Get the current time.
        now = self.get_clock().now()
        # Get the duration since the last odometry messoge was received.  
        delta = int((now - self._odometry_received_timestamp).nanoseconds / 1000000000)
        if (delta > 1):
            self.get_logger().warn(f'Waiting to receive odometry messages. ({delta -1} seconds)')
    
    # ..............................................................
    # Odometry subscriber message callback.
    def _odometry_subscriber_callback(self, odometry_msg: OdometryMessage):
        # Update the time stamp.
        self._odometry_received_timestamp = self.get_clock().now()
        # Get orientation quaternion
        quat = Quaternion.from_msg(odometry_msg.pose.pose.orientation)

        # Get the robots yaw in degrees
        yaw = quat.get_yaw(True)

        # Get the coordinate position of the robot.
        x_position = odometry_msg.pose.pose.position.x
        y_position = odometry_msg.pose.pose.position.y

        self.get_logger().info(f"Yaw: {yaw:.3f} (deg) X: {x_position:.3f} Y: {y_position:.3f}")

    #...............................................................
