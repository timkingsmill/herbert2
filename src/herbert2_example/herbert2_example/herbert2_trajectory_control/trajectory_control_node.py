from rclpy.node import Node
from geometry_msgs.msg import Point

class TrajectoryControlNode(Node):

    def __init__(self):
        super().__init__('trajectory_control_node')
        self.get_logger().info("Created a TrajectoryControlNode.")

