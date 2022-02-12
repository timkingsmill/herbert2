#!/usr/bin/env python3

# -----------------------------------------------------------------------------

import rclpy

from herbert2_example.herbert2_position_control.herbert2_position_control \
    import Herbert2PositionControl

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        herbert2_position_control = Herbert2PositionControl()
        try:
            rclpy.spin(herbert2_position_control)
        except KeyboardInterrupt:
            pass
        finally:    
            herbert2_position_control.destroy_node()
    finally:
        rclpy.shutdown()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
