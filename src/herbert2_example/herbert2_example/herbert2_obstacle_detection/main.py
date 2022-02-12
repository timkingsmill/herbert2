# -----------------------------------------------------

import rclpy.executors
from . import Herbert2ObstacleDetection

# -----------------------------------------------------

def main(args=None):
    print('Hello from the Obstacle Detection Example')
    rclpy.init(args=args)
    try:
        # Runs all callbacks in the main thread
        executor = rclpy.executors.SingleThreadedExecutor()
        node = Herbert2ObstacleDetection()
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


# -----------------------------------------------------
# -----------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------
# -----------------------------------------------------
