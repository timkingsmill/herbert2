import threading
import queue
from time import sleep

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry as OdometryMessage
from sensor_msgs.msg import Imu as ImuMessage
from sensor_msgs.msg import JointState as JointStateMessage

from herbert2_geometry import Quaternion
from herbert2_geometry.robot_maths import get_robot_position_from_wheel_position

# .............................................................................

class OdometryNode(Node): 
    """
    Subscribes to:
        Imu messages.           Topic: /imu_data.

        JointState messages.    Topic: /joint_states
            Motors: Axis1, Axis2 and Axis3 joint positions.

    Publishes: 
        Odometry messages. Topic /odom  

    """

    def __init__(self) -> None:
        """ Herbert2 Robot Odometry Node """
        super().__init__(node_name = 'herbert2_odometry')
        self._connected = False

        self._imu_data_queue = queue.Queue()
        self._joint_state_queue = queue.Queue()

        self._imu_yaw_offset: float = 0.0
        self._imu_yaw_offset_initialized: bool = False
        
        # The relative joint / motor positions.
        self._diff_joint_positions = [0.0, 0.0, 0.0]

        # Cache the most recent positions of the joints / motors.
        self._last_joint_positions = [0.0, 0.0, 0.0]

        self._robot_pose = [0.0, 0.0, 0.0]
        
        self._joint_state_sub = self.create_subscription(
            JointStateMessage,
            '/joint_states',
            self._joint_state_callback,
            qos_profile = qos_profile_sensor_data
        )

        self._joint_state_time = self.get_clock().now()

        self._imu_data_sub = self.create_subscription(
            ImuMessage,
            '/imu_data',
            self._imu_data_callback,
            qos_profile = qos_profile_sensor_data
        )

        self._imu_data_time = self.get_clock().now()
        
        # Initialise timers
        self._check_subscribers_timer = self.create_timer(
            1.0,  # unit: second
            self._check_subscribers_timer_callback)

        # Initialize publishers
        self._odom_publisher = self.create_publisher(
            OdometryMessage, 'odometry', qos_profile = qos_profile_sensor_data)

        self._exec_thread = threading.Thread(target = self._exec_update_odometry)
        self._exec_thread.daemon = False
        self._exec_thread.name = 'odometry-executer'

        self._connected = True
        self._exec_thread.start()

        self.get_logger().info('Odometry node constructor OK')

    # ..............................................................

    def destroy_node(self):
        self._connected = False

        super().destroy_node()
        self.get_logger().info('Destroyed the odometry node')
    
    # ..............................................................

    def _check_subscribers_timer_callback(self):
        # Get the current time.
        now = self.get_clock().now()

        # Get the duration since the last IMU messoge was received.  
        imu_delta = int((now - self._imu_data_time).nanoseconds / 1000000000)
        # If the duration was more than 1 second the report that 
        # the IMU subscriber is waiting to receive IMU messages 
        if (imu_delta > 1):
            self.get_logger().warn(f'Waiting to receive IMU messages. ({imu_delta -1} seconds)')

        # Get the duration since the last JointState messoge was received.  
        joint_state_delta = int((now - self._joint_state_time).nanoseconds / 1000000000)
        # If the duration was more than 1 second the report that 
        # the JointState subscriber is waiting to receive JointState messages 
        if (joint_state_delta > 1):
            self.get_logger().warn(f'Waiting to receive JointState messages. ({joint_state_delta -1} seconds)')

    # ..............................................................

    def _imu_data_callback(self, imu_data_msg: ImuMessage):
        self._imu_data_time = self.get_clock().now()
        if (self._connected):
            self._imu_data_queue.put(imu_data_msg)
 
    # ..............................................................

    def _joint_state_callback(self, joint_state_msg: JointStateMessage):
        self._joint_state_time = self.get_clock().now()
        if (self._connected):
            self._joint_state_queue.put(joint_state_msg)

    # ..............................................................

    def _exec_update_odometry(self) -> None:
        while (self._connected):
            joint_state_msg: JointStateMessage = self._get_joint_state_msg()
            imu_data_msg: ImuMessage = self._get_imu_data_msg()

            if (isinstance(joint_state_msg, JointStateMessage) & isinstance(imu_data_msg, ImuMessage)): 
                
                imu_nano: int = imu_data_msg.header.stamp.nanosec
                joint_state_nano: int = joint_state_msg.header.stamp.nanosec
                nano_nano_secs: float = abs(imu_nano - joint_state_nano) / 1000000000.0

                if (nano_nano_secs < 1.0):
                    if (self._update_imu(imu_data_msg)):
                        #self.get_logger().info(str(nano_nano_secs))
                        if (self._update_joint_state(joint_state_msg)):
                            #self.get_logger().info(f'Yaw: {self._imu_yaw:.3f}  Offset: {self._imu_yaw_offset:.3f}')
                            self._calculate_odometry(0)
                            self._publish_odometry()

            else:
                sleep(0.010)  # 100 Hz

    # ..............................................................

    def _get_joint_state_msg(self) -> JointStateMessage:
        if (not self._joint_state_queue.empty()):
            joint_state_msg: JointStateMessage = self._joint_state_queue.get()
            return joint_state_msg
        else:
            return None

    # ..............................................................

    def _get_imu_data_msg(self) -> ImuMessage:
        if (not self._imu_data_queue.empty()):
            imu_data_msg: ImuMessage = self._imu_data_queue.get()
            return imu_data_msg
        else:
            return None

    # ..............................................................

    def _calculate_odometry(self, duration: Duration):
        """
            parm duration in nanoseconds.

            Convert the a, b and c motor position vectors to X and Y robot positions.

            Polar Robot Movement:
                +X: Forward
                -X: Reverse 
                +Y: Right
                -Y: Left
        """
        # Cache the position of each wheel in units of linear meters. 
        a_pos: float = self._last_joint_positions[0]
        b_pos: float = self._last_joint_positions[1]
        c_pos: float = self._last_joint_positions[2]

                # Convert the a, b and c motor position vectors to X and Y robot positions.
        robot_pos = get_robot_position_from_wheel_position(a_pos, b_pos, c_pos)
        #self.get_logger().info(f'Robot Location X = {robot_pos[0]:.3f}  Y = {robot_pos[1]:.3f}')

        self._robot_pose[0] = robot_pos[0]  
        self._robot_pose[1] = robot_pos[1]  
        self._robot_pose[2] = self._imu_yaw

    # ..............................................................

    def _publish_odometry(self):
        """
            Publish odometry
        """
        time_stamp = self.get_clock().now().to_msg()
        odometry_msg = OdometryMessage()

        odometry_msg.header.frame_id = 'odometry'
        odometry_msg.header.stamp = time_stamp
        odometry_msg.child_frame_id = 'footprint_link'

        odometry_msg.pose.pose.position.x = self._robot_pose[0]
        odometry_msg.pose.pose.position.y = self._robot_pose[1]
        odometry_msg.pose.pose.position.z = 0.0   

        # Create a quaternion to represent YAW in degrees.
        # NOTE: Don't use ROLL or PITCH 
        quat = Quaternion.from_euler(0.0, 0.0, self._robot_pose[2], use_degrees = True)

        odometry_msg.pose.pose.orientation.x = quat.x
        odometry_msg.pose.pose.orientation.y = quat.y
        odometry_msg.pose.pose.orientation.z = quat.z
        odometry_msg.pose.pose.orientation.w = quat.w

        # Publish the odometry message.
        self._odom_publisher.publish(odometry_msg)

        # ------------------------------------------
        # Broadcast odometry frame transform
        odometry_tf_msg = TransformStamped()

        odometry_tf_msg.header.frame_id = 'odometry'
        odometry_tf_msg.header.stamp = time_stamp
        odometry_tf_msg.child_frame_id = 'footprint_link'

        # ------------------------------------------
        # Publish and broadcast odometry.
        #self._tf_broadcaster.sendTransform(odometry_tf)

    # ..............................................................

    def _update_joint_state(self, joint_state_msg: JointStateMessage) -> bool:
        """
        Update the relative joint positions.
        Calculate the differential or new position relative to the last or previous position.
        """
        if (self._imu_yaw_offset_initialized == True):
            # Cache the current position of each wheel.
            a_pos = joint_state_msg.position[0] #/ 3.06
            b_pos = joint_state_msg.position[1] #/ 3.06
            c_pos = joint_state_msg.position[2] #/ 3.06

            # Update the relative joint position.
            # (The position change since recieving the previous joint positions )
            self._diff_joint_positions[0] = a_pos - self._last_joint_positions[0]
            self._diff_joint_positions[1] = b_pos - self._last_joint_positions[1]
            self._diff_joint_positions[2] = c_pos - self._last_joint_positions[2]

            # Cache the current position of each wheel with the current joint positions.
            self._last_joint_positions[0] = a_pos
            self._last_joint_positions[1] = b_pos
            self._last_joint_positions[2] = c_pos

            return True
        else:
            return False

    # ..............................................................

    def _update_imu(self, imu_msg: ImuMessage) -> bool:
        """
            Update the yaw in degrees. (-180.0 .. 180.0 degrees)
            Positive yaw is an anti-clockwise rotation of the robot.
            Negative yaw is a clockwise rotation of the robot
        """
        quaternion: Quaternion = Quaternion.from_msg(imu_msg.orientation)
        if not (quaternion.is_valid()):
            return False
        
        if (self._imu_yaw_offset_initialized == False):
            # Update the yaw offset (-180.0 .. 180.0 degrees)
            self._imu_yaw_offset = quaternion.get_yaw(True)
            self._imu_yaw_offset_initialized = True
            return False
        else:
            # Yaw offset is initialized
            # get the yaw in degrees. (-180.0 .. 180.0 degrees)
            # Anti clockwise = positive yaw.
            # Clockwise = negative yaw
            raw_yaw : float = quaternion.get_yaw(True)

            # apply the yaw offset.
            self._imu_yaw = raw_yaw + self._imu_yaw_offset
            if (self._imu_yaw > 180.0):
                self._imu_yaw = self._imu_yaw - 360.0
            elif (self._imu_yaw < -180.0):
                self._imu_yaw = self._imu_yaw + 360.0

            return True

    # ..............................................................
    # ..............................................................
