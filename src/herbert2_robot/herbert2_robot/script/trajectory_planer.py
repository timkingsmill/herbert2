import re
import numpy as np
import weakref

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdometryMessage

#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from .trajectory_controller import TrajectoryController

from .motor_controller import MotorController
from .trajectory import Trajectory, TrajectoryStep
from herbert2_odometry.kinematics import robot_frame_to_wheel_frame_velocity 

# .................................................................................

class TrajectoryPlaner:

    # .................................................................................

    def __init__(self, controller: MotorController) -> None:
        self._node_ref = weakref.ref(controller.node)
        self._controller = controller
        self._twist_msg = Twist() 
        self._trajectory = Trajectory()

        
        # Initialise subscribers
        # Listen for messages describing the target velocity.
        self._velocity_sub = self._node_ref().create_subscription(Twist, 
                            'cmd_twist_raw', 
                            self._velocity_subscriber_callback, 
                            qos_profile = QoSProfile(depth=10))

        # Subscribe to odometry messages
        self._odometry_sub = self._node_ref().create_subscription(
                            OdometryMessage,
                            '/odometry',
                            self._odometry_subscriber_callback,
                            qos_profile = QoSProfile(depth=10)
        )

    # .................................................................................

    def _velocity_subscriber_callback(self, twist_msg: Twist):
        if (self._twist_msg != twist_msg):
            self._twist_msg = twist_msg

            magnitude = np.linalg.norm([twist_msg.linear.x, twist_msg.linear.y])
            if (magnitude > 0.0):

                left, rear, right = robot_frame_to_wheel_frame_velocity(twist_msg.linear.x, twist_msg.linear.y)
                print(left, rear, right)

                vmax = 4.0 #self._controller.get_max_velocity()
                amax = 4.0 #self._controller.get_max_acceleration()
                dmax = 4.0 #self._controller.get_max_deceleration()
                initial_position = 0.0  #self._controller.get_current_position()
                intial_velocity = 0.0
                goal = magnitude

                if (self._trajectory.plan_trajectory(goal, initial_position, intial_velocity, vmax, amax, dmax)):

                    time: float = 0.0
                    time_interval: float = 0.001   #0.000125

                    while (time < self._trajectory.total_time):
                        step: TrajectoryStep = self._trajectory.evaluate(time)
                        #print(step.velocity)
                        self._controller.set_input_velocity(step.velocity * left, step.velocity * rear, step.velocity * right)
                        time += time_interval

            self._controller.set_input_velocity(0.0, 0.0, 0.0)

       
    # .................................................................................

    def _odometry_subscriber_callback(self, msg: OdometryMessage):
        pass

    # .................................................................................

    def plan_trajectory(self, pose_start, pose_end, pose_step):
        pass

    # .................................................................................
