# ---------------------------------------------------------------

import rclpy.executors
from .script import Herbert2Robot

# ---------------------------------------------------------------
# ---------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        # Runs all callbacks in the main thread
        executor = rclpy.executors.SingleThreadedExecutor()
        robot = Herbert2Robot()
        try:
            try:
                # Add nodes to this executor
                executor.add_node(robot)
                executor.spin()
            except KeyboardInterrupt:
                pass
        finally:   
            robot.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()
    
# ---------------------------------------------------------------
# ---------------------------------------------------------------

if __name__ == '__main__':
    main()

# ---------------------------------------------------------------
# ---------------------------------------------------------------


