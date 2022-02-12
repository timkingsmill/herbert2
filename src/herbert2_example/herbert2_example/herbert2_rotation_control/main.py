import rclpy

from .rotation_control import RotationControl

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        rotation_control = RotationControl()
        try:
            rclpy.spin(rotation_control)
        except KeyboardInterrupt:
            pass
        finally:    
            rotation_control.destroy_node()
    finally:
        rclpy.shutdown()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
