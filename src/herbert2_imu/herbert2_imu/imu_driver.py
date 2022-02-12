import rclpy.executors
from .imu_driver_node import IMUDriverNode

def main(args=None):
    print('Hi from herbert2_imu ROS2 node.')

    rclpy.init(args=args)
    try:
        # Runs all callbacks in the main thread
        executor = rclpy.executors.SingleThreadedExecutor()

        node = IMUDriverNode()
        try:
            try:
                # Add nodes to this executor
                executor.add_node(node)
                executor.spin()
            except KeyboardInterrupt:
                pass
        finally:   
            node.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
