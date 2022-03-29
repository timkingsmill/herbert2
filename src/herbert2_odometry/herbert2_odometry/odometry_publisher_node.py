from cmath import pi
import threading
import queue
import math
from time import sleep

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry as OdometryMessage
from sensor_msgs.msg import Imu as ImuMessage
from sensor_msgs.msg import JointState as JointStateMessage

from herbert2_geometry import Quaternion
import kinematics

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
        self._debug_output_counter = 0
        # Queue IMU messages
        self._imu_data_queue = queue.Queue()
        # Queue Joint State messages
        self._joint_state_queue = queue.Queue()
        
        self._imu_yaw_raw: float = 0.0 
        self._imu_yaw_offset: float = 0.0
        self._imu_yaw_offset_initialized: bool = False
        
        # The relative joint / motor positions.
        self._diff_joint_positions = [0.0, 0.0, 0.0]

        # Cache the most recent positions of the joints / wheels.
        # 
        self._absolute_wheel_positions = [0.0, 0.0, 0.0]

        self._robot_pose = [0.0, 0.0, 0.0]
        
        # Subscribe to Joint State messages
        self._joint_state_sub = self.create_subscription(
            JointStateMessage,
            '/joint_states',
            self._joint_state_callback,
            qos_profile = qos_profile_sensor_data
        )
        self._joint_state_time = self.get_clock().now()

        # Subscribe to IMU messages
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
            OdometryMessage, '/odometry', qos_profile = qos_profile_sensor_data)

        from tf2_ros import TransformBroadcaster
        self._tf_broadcaster = TransformBroadcaster(self)


        # Calculate and publish odometry data in a thread.
        self._exec_thread = threading.Thread(target = self._update_odometry_thread_proc)
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
    # Check that the message subscribers are receiving messages.
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
    # Queue incomming IMU messages.
    def _imu_data_callback(self, imu_data_msg: ImuMessage):
        self._imu_data_time = self.get_clock().now()
        if (self._connected):
            self._imu_data_queue.put(imu_data_msg)
 
    # ..............................................................
    # Queue incomming Joint State messages.
    def _joint_state_callback(self, joint_state_msg: JointStateMessage):
        self._joint_state_time = self.get_clock().now()
        if (self._connected):
            self._joint_state_queue.put(joint_state_msg)

    # ..............................................................
    # Thread procedure 
    def _update_odometry_thread_proc(self) -> None:
        while (self._connected):
            # Retrive queued messages.
            joint_state_msg: JointStateMessage = self._get_joint_state_msg()
            imu_data_msg: ImuMessage = self._get_imu_data_msg()

            # Test that retrived message types are correct.
            if (isinstance(joint_state_msg, JointStateMessage) & isinstance(imu_data_msg, ImuMessage)): 
                imu_nano: int = imu_data_msg.header.stamp.nanosec
                joint_state_nano: int = joint_state_msg.header.stamp.nanosec
                nano_nano_secs: float = abs(imu_nano - joint_state_nano) / 1000000000.0
                if (nano_nano_secs < 1.0):
                    if (self._parse_imu_msg(imu_data_msg)):
                        if (self._parse_joint_state_msg(joint_state_msg)):
                            self._calculate_odometry(0)
                            self._publish_odometry()
            else:
                sleep(0.010)  # 100 Hz

    # ..............................................................
    # Get Joint State messages from the queue.
    def _get_joint_state_msg(self) -> JointStateMessage:
        if (not self._joint_state_queue.empty()):
            joint_state_msg: JointStateMessage = self._joint_state_queue.get()
            return joint_state_msg
        else:
            return None

    # ..............................................................
    # Get IMU data messages from the queue.
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
                +Y: Forward
                -Y: Reverse 
                
                +X: Right
                -X: Left
        """

        # Cache the position of each wheel in units of linear meters. 
        a_pos: float = self._absolute_wheel_positions[0]
        b_pos: float = self._absolute_wheel_positions[1]
        c_pos: float = self._absolute_wheel_positions[2]

        x, y = kinematics.wheel_frame_to_robot_frame(a_pos, b_pos, c_pos)

        ###x = -(((2 * b_pos) - a_pos - c_pos) / 3.0)
        ###y = ((math.sqrt(3) * a_pos) - (math.sqrt(3) * c_pos)) / 3.0
            
        # Radius of the robot = 0.145
        # radians = 0 .. 2pi
        #radians = (a_pos + b_pos + c_pos) / (3.0 * pi * 0.145)
        #radius = 0.145 # too small
        radius = 0.1458
        radians = (a_pos + b_pos + c_pos) / (3.0 * radius)

        # yaw = -180 .. 180
        robot_yaw = 180.0 - (math.degrees(radians) % 360.0)

        # Calculate the IMU sensor YAW 
        # Scale the raw yaw to 0 .. 360.0
        self._imu_yaw_raw = self._imu_yaw_raw % 360.0
        # Calculate adjusted Yaw. (-180 .. 180) Robot forward = 0.0 yaw. 
        self._imu_yaw = ((self._imu_yaw_raw + self._imu_yaw_offset) % 360.0) - 180.0

        """
        # slow down the log output.
        self._debug_output_counter = self._debug_output_counter + 1
        if (self._debug_output_counter > 40):
            self._debug_output_counter = 0
            self.get_logger().info(f'Motor Odometry:')
            #self.get_logger().info(f'    Position:')
            #self.get_logger().info(f'        A: {a_pos:.3f} B: {b_pos:.3f}  C: {c_pos:.3f}') 
            self.get_logger().info(f'    Robot Frame:')
            self.get_logger().warn(f'        IMU Yaw:           {self._imu_yaw:.3f}') 
            self.get_logger().warn(f'        Robot Yaw:         {robot_yaw:.3f}') 
            #self.get_logger().info('')
            #self.get_logger().warn(f'        Measured Offset:   {(self._imu_yaw - robot_yaw):.3f}') 
            self.get_logger().info('')
        """

        #self.get_logger().info(f'Robot Location X = {robot_pos[0]:.3f}  Y = {robot_pos[1]:.3f}')

        self._robot_pose[0] = x  
        self._robot_pose[1] = y  
        self._robot_pose[2] = self._imu_yaw

        #self.get_logger().info(f'Robot Pose = {self._robot_pose}')

    # ..............................................................

    def _parse_joint_state_msg(self, joint_state_msg: JointStateMessage) -> bool:
        """
        Update the relative joint positions.
        Calculate the differential or new position relative to the last or previous position.
        """
        if (self._imu_yaw_offset_initialized == True):
            # Cache the current position of each wheel.
            # Message data - 1.0 = 360 degrees.
            ###### Convert message data to linear meters. 3.141 rotations per metre.
            a_pos = joint_state_msg.position[0] / 3.141
            b_pos = joint_state_msg.position[1] / 3.141
            c_pos = joint_state_msg.position[2] / 3.141

            # Update the relative joint position.
            # (The position change since recieving the previous wheel positions )
            self._diff_joint_positions[0] = a_pos - self._absolute_wheel_positions[0]
            self._diff_joint_positions[1] = b_pos - self._absolute_wheel_positions[1]
            self._diff_joint_positions[2] = c_pos - self._absolute_wheel_positions[2]

            # Cache the current position of each wheel with the current joint or wheel positions.
            self._absolute_wheel_positions[0] = a_pos
            self._absolute_wheel_positions[1] = b_pos
            self._absolute_wheel_positions[2] = c_pos

            return True
        else:
            return False

    # ..............................................................

    def _parse_imu_msg(self, imu_msg: ImuMessage) -> bool:
        """
            Update the yaw in degrees. (-180.0 .. 180.0 degrees)
            Positive yaw is an anti-clockwise rotation of the robot.
            Negative yaw is a clockwise rotation of the robot
        """
        quaternion: Quaternion = Quaternion.from_msg(imu_msg.orientation)
        if not (quaternion.is_valid()):
            return False
        
        if (self._imu_yaw_offset_initialized == False):
            # Update the yaw offset from : Yaw = -180.0 .. 180.0 degrees
            # to : Yaw = 0.0 .. 360.0 degrees
            self._imu_yaw_offset = -1 * quaternion.get_yaw(True)
            self._imu_yaw_offset_initialized = True
            return False
        else:
            # Yaw offset is initialized
            # quaternion yaw in degrees. (-180.0 .. 180.0 degrees)
            # Anti clockwise = positive yaw. Clockwise = negative yaw.
            # Scale raw yaw to : 0 .. 360.
            self._imu_yaw_raw : float = quaternion.get_yaw(True)
            return True

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

        # Publish the raw omni wheel positions in metres.
        odometry_msg.twist.twist.linear.x = self._absolute_wheel_positions[0]
        odometry_msg.twist.twist.linear.y = self._absolute_wheel_positions[1]
        odometry_msg.twist.twist.linear.z = self._absolute_wheel_positions[2]

        # Publish the X and Y position of the robot in metres.
        odometry_msg.pose.pose.position.x = self._robot_pose[0]
        odometry_msg.pose.pose.position.y = self._robot_pose[1]
        odometry_msg.pose.pose.position.z = 0.0   

        # Create a quaternion to represent YAW in degrees.
        # NOTE: Don't use ROLL or PITCH 
        quat = Quaternion.from_euler(0.0, 0.0, self._robot_pose[2], use_degrees = True)

        #self.get_logger().error(f'  quat:                       {quat}') 

        odometry_msg.pose.pose.orientation.x = quat.x
        odometry_msg.pose.pose.orientation.y = quat.y
        odometry_msg.pose.pose.orientation.z = quat.z
        odometry_msg.pose.pose.orientation.w = quat.w

        #self.get_logger().info(f"Publisher Orientation: {odometry_msg.pose.pose.orientation}")

        # Publish the odometry message.
        self._odom_publisher.publish(odometry_msg)

        # ------------------------------------------
        # Broadcast odometry frame transform
        odometry_tf_msg = TransformStamped()

        odometry_tf_msg.header.frame_id = 'odometry'
        odometry_tf_msg.header.stamp = time_stamp
        odometry_tf_msg.child_frame_id = 'footprint_link'

        odometry_tf_msg.transform.translation.x = odometry_msg.pose.pose.position.x
        odometry_tf_msg.transform.translation.y = odometry_msg.pose.pose.position.y
        odometry_tf_msg.transform.translation.z = odometry_msg.pose.pose.position.z
        odometry_tf_msg.transform.rotation = odometry_msg.pose.pose.orientation

        # ------------------------------------------
        # Publish and broadcast odometry transform.
        self._tf_broadcaster.sendTransform(odometry_tf_msg)

    # ..............................................................


    # ..............................................................
    # ..............................................................
