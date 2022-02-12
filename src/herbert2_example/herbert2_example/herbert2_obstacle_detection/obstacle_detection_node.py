# ---------------------------------------------------------------------------

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------

class Herbert2ObstacleDetection(Node):
    
    # -----------------------------------------------------------------------

    def __init__(self):
        super().__init__('herbert2_obstacle_detection')

        """
            Initialise variables
        """
        self._linear_velocity_x = 0.0  # unit: m/s
        self._linear_velocity_y = 0.0  # unit: m/s
        self._linear_velocity_z = 0.0  # unit: m/s
        
        self._angular_velocity = 0.0  # unit: m/s
        self._scan_ranges = []
        self._init_scan_state = False  # To get the initial scan data at the beginning

        self._stopped = False
        """
            Initialise ROS publishers and subscribers
        """
        ####qos = QoSProfile(depth=10)

        """
            Initialise publishers
        """
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile = qos_profile_sensor_data)

        """
            Initialise subscribers
        """
        self.scan_sub = self.create_subscription(
            LaserScan,
            'rplidar_scan',
            self._scan_callback,
            qos_profile = qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self._cmd_vel_raw_callback,
            qos_profile = qos_profile_sensor_data)

        """
            Initialise timers
        """
        self._update_timer = self.create_timer(
            0.020,  # unit: s
            self._update_timer_callback)

        # all ok
        self.get_logger().info("Herbert2 obstacle detection node has been initialised.")

    # -----------------------------------------------------------------------

    def _cmd_vel_raw_callback(self, msg):
        self._linear_velocity_x = msg.linear.x
        self._linear_velocity_y = msg.linear.y
        self._linear_velocity_z = msg.linear.z

        self._angular_velocity = msg.angular.z

    # -----------------------------------------------------------------------

    def _detect_obstacle(self):
        twist = Twist()
        obstacle_distance = min(self._scan_ranges)
        safety_distance = 0.3  # unit: m

        if obstacle_distance > safety_distance:
            if (self._stopped == True):
                self.get_logger().info(f"Obstacles avoided. Robot restarted.")

            self._stopped = False
            
            twist.linear.x = self._linear_velocity_x
            twist.linear.y = self._linear_velocity_y
            twist.linear.z = self._linear_velocity_z

            twist.angular.z = self._angular_velocity
            
        else:
            if (self._stopped == False):
                self.get_logger().info(f"Obstacles are detected nearby {obstacle_distance}. Robot stopped.")

            self._stopped = True

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.z = 0.0

        self._cmd_vel_pub.publish(twist)

    # -----------------------------------------------------------------------

    def _scan_callback(self, msg):
        self._scan_ranges = msg.ranges
        self._init_scan_state = True

    # -----------------------------------------------------------------------

    def _update_timer_callback(self):
        if self._init_scan_state == True:
            self._detect_obstacle()

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------

