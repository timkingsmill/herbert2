import rclpy

from .trajectory_control_node import TrajectoryControlNode
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        trajectory_control_node = TrajectoryControlNode()
        try:
            rclpy.spin(trajectory_control_node)
        except KeyboardInterrupt:
            pass
        finally:    
            trajectory_control_node.destroy_node()
    finally:
        rclpy.shutdown()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
