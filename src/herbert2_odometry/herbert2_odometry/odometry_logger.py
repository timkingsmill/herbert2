import rclpy.executors
from .odometry_logger_node import OdometryLoggerNode

#...............................................................

def main(args=None):
    print('Hi from herbert2_odometry_logger ROS2 node.')

    rclpy.init(args=args)
    try:
        # Runs all callbacks in the main thread
        executor = rclpy.executors.SingleThreadedExecutor()
        node = OdometryLoggerNode()
        try:
            try:
                # Add node to the executor
                executor.add_node(node)
                executor.spin()
            except KeyboardInterrupt:
                pass
        finally:   
            node.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()

#...............................................................

if __name__ == '__main__':
    main()

#...............................................................