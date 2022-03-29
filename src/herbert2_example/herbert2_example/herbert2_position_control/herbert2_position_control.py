#!/usr/bin/env python3
# -----------------------------------------------------------------------

"""
    ***********************************************
    SHOULD BE A Odometery MESSAGE SUBSCRIBER.
    MOT an IMU message SUBSCRIBER
    D:\ros2-turtlebot3\turtlebot3\turtlebot3_example\turtlebot3_example\turtlebot3_position_control
    ***********************************************
"""

import sys
import termios
import threading
import time
import math

from herbert2_geometry import Quaternion
from . herbert2_path import Herbert2Path

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# -----------------------------------------------------------------------
# -----------------------------------------------------------------------
from herbert2_geometry import Quaternion

class Herbert2PositionControl(Node):

    # -------------------------------------------------------------------

    def __init__(self):
        super().__init__('herbert2_position_control')
        self.get_logger().info('Initialize Herbert2 position control ROS node.')
        self._logging_enabled = True
        self._mutex = threading.Lock()
        
        # discard the first 50 odometry messages received
        self._ignore_msg = 50

        self._current_pose_x = 0.0
        self._current_pose_y = 0.0
        self._current_pose_theta = 0.0

        self._target_pose_x = 0.0
        self._target_pose_y = 0.0
        self._target_pose_theta = 0.0

        self.step = 1
        self.get_key_state = False
        self._odom_message_ready = False  # An odometry message is available.

        """
        Initialise ROS publishers and subscribers
        """
        # Initialise publishers
        # Publish Twist geometry messages.
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            qos_profile = qos_profile_sensor_data)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry',
            self._odom_message_callback,
            qos_profile = qos_profile_sensor_data
        )

        self.get_logger().info('Position control node waiting for odometry messages.')

        # Initialise the update timer.
        # 100 Hz should be quick enough.
        #self.update_timer = self.create_timer(0.010, self._update_timer_callback) 

        self._update_thread_running = False
        self._update_thread_terminated = False
        self._update_thread = threading.Thread(target = self._update_thread_proc)
        self._update_thread.daemon = True
        self._update_thread.name = 'update-position-thread'

    # -------------------------------------------------------------------

    def destroy_node(self):
        """
            Overridden to gracefully terminate the update thread.
        """
        if (self._update_thread_running is True):
            self._update_thread_running = False
            while (self._update_thread_terminated is False):
                # Waiting for the thread to terminate
                time.sleep(0.010)
        self.get_logger().info('Destroy herbert2_position_control')
        super().destroy_node()

    # -------------------------------------------------------------------

    def _update_thread_proc(self):
        """
        Async thread proceedure.
        """
        while (self._update_thread_running is True):
            time.sleep(0.010)
            self._generate_path()

        self._log_info('Thread Loop Terminated')
        self._update_thread_terminated = True
        

    # -------------------------------------------------------------------

    def _log_info(self, message: str) -> None: 
        if (self._logging_enabled):
            self.get_logger().info(message)

    # -------------------------------------------------------------------

    @staticmethod
    def _calc_target_pose_angle(start: float, offset: float) -> float:
        angle: float = start + offset
        if (angle >= 180.0):
            angle = angle - 360.0
        elif (angle <= -180.0):
            angle = angle + 360.0
        return angle
       
    # -------------------------------------------------------------------

    @staticmethod
    def _calc_rotation_angle(start: float, target: float) -> float:
        angle: float = target - start
        if (angle >= 180.0):
            angle = angle - 360.0
        elif (angle <= -180.0):
            angle = angle + 360.0
        return angle

    # -------------------------------------------------------------------

    def _generate_path(self):
        """
        Given the odometry data and goal pose, calculate the path to the goal pose
        and then publish the velocity message required by the robot to reach that goal. 

        This method is executed in the update thread context.
        """
        twist = Twist()

        if self.get_key_state is False:
            # Get positional target pose from the user.
            input_x, input_y, input_theta = self._get_key()

            self._mutex.acquire()
            try:
                # Calculate the robot's target linear goal pose. (x, y)
                self._target_pose_x = self._current_pose_x + input_x
                self._target_pose_y = self._current_pose_y + input_y

                # Calculate the robot's target angular goal pose.
                self._target_pose_theta = self._calc_target_pose_angle(self._current_pose_theta, input_theta)    
            finally:
                self._mutex.release()

            # Print a summary of the requested path
            summary = f"""

    Herbert Path
    ------------------------------------------
    Current Yaw: {self._current_pose_theta:.3f}
    Goal Yaw:    {self._target_pose_theta:.3f}
    ------------------------------------------

    """
            print(summary)
            # Set the key state to start the robots movement.
            self.get_key_state = True

        else:
            # Step 1: Rotate
            if self.step == 1:
                # Get the remaining angle required to travel to the goal angular pose.

                angle: float = self._calc_rotation_angle(self._current_pose_theta, self._target_pose_theta)
                print(f'Current Pose: {self._current_pose_theta:.3f} Target Pose: {self._target_pose_theta:.3f} Rotation Angle: {angle:.3f}')
                angular_velocity = 0.4  # unit: ????????
                twist, self.step = Herbert2Path.rotate(angle, angular_velocity, self.step)

            # Step 2: Go Straight
            elif self.step == 2:
                #print("LINEAR")
                
                x_distance = self._target_pose_x - self._current_pose_x
                y_distance = self._target_pose_y - self._current_pose_y
                twist, self.step = Herbert2Path.go_straight(x_distance, y_distance, self.step)
                
                #linear_velocity = 0.1  # unit: m/s
                #twist, self.step = Herbert2Path.go_straight(x_distance, y_distance, linear_velocity, self.step)

            # Step 3: Rotate
            elif self.step == 3:
                self.step = 4

                """
                # Get the remaining angle required to travel to the goal angular pose.
                angle: float = self._calc_rotation_angle(self._current_pose_theta, self._target_pose_theta)
                print(f'Current Pose: {self._current_pose_theta:.3f} Target Pose: {self._target_pose_theta:.3f} Rotation Angle: {angle:.3f}')
                # Slow down at the end of the rotation.
                angular_velocity = 0.2  # unit: ????????
                twist, self.step = Herbert2Path.rotate(angle, angular_velocity, self.step)
                """

            # Reset
            elif self.step == 4:
                twist.angular.z = 0.0
                self.get_key_state = False
                self.step = 1

        # Publish the twist message every time _generate_path() is called.
        self.cmd_vel_pub.publish(twist)

    # -----------------------------------------------------------------------
    # -----------------------------------------------------------------------

    def _odom_message_callback(self, msg: Odometry):
        """
            Proccess the odometry message.
        """
        self._ignore_msg = self._ignore_msg - 1
        if (self._ignore_msg <= 0):

            #self._log_info(f'ODOM Message Called')

            # Get the relevent odometry data from the message.
            quat: Quaternion = Quaternion.from_msg(msg.pose.pose.orientation)
            if (quat.is_valid):
                yaw: float = quat.get_yaw(use_degrees = True)

                self._mutex.acquire()
                try:
                    # Cache the current robots pose.
                    self._current_pose_x = msg.pose.pose.position.x
                    self._current_pose_y = msg.pose.pose.position.y
                    self._current_pose_theta = yaw

                    # Set flag to indicate an odometry message has been received.
                    self._odom_message_ready = True

                finally:
                    self._mutex.release()

                """
                current = self._current_pose_theta
                target = self._calc_target_pose_angle(self._current_pose_theta, 30)    
                angle  = self._calc_rotation_angle(current, target)
                print(f'Current Yaw: {current:.3f}   Target: {target:.3f}   Angle: {angle:.3f}')
                """


                if (self._update_thread_running is False):
                    self._update_thread_running = True
                    self._update_thread.start()

    # -----------------------------------------------------------------------

    def _get_key(self):
        # Print terminal message and get inputs

        terminal_msg = f"""
    Herbert2 Position Control
    ---------------------------------------------------------
    Current Yaw: {self._current_pose_theta:.3f}

    Enter
        x: goal position x (unit: m)
        y: goal position y (unit: m)
        theta: goal orientation (range: -180 .. 180, unit: deg)

    All goal values are relative to the current pose.
    ---------------------------------------------------------
    """
        self._logging_enabled = False
        try:
            print(terminal_msg)
            input_x = float(input("Input x: "))
            input_y = float(input("Input y: "))
            input_theta = float(input("Input theta: "))
            while input_theta > 179.0 or input_theta < -179.0:
                self.get_logger().info("Enter a value for theta between -180 and 180")
                input_theta = float(input("Input theta: "))

            print("")
            # Calculate the robot's target angular goal pose.
            pose_theta = self._calc_target_pose_angle(self._current_pose_theta, input_theta)    
            print(f"Target Theta: {pose_theta}")


            input("Hit any key to continue.......")

            # I don't know what this does, but it seems to be OK.
            settings = termios.tcgetattr(sys.stdin)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        finally:    
            self._logging_enabled = True

        return input_x, input_y, input_theta


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------

