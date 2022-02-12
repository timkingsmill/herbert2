import math

from geometry_msgs.msg import Twist

class Herbert2Path():
    

    def rotate(angle, angular_velocity, step):
        """

        """
        twist: Twist = Twist()

        # Don't rotate small angles
        if math.fabs(angle) >= 0.5:
            # Set velocity to rotate clockwise or counter clockwise.
            if (angle > 0.0):
                twist.angular.z = angular_velocity
            elif (angle < 0.0):
                twist.angular.z = -angular_velocity
        else:
            # Finished. The robot has reached the target angle.
            twist.angular.z = 0.0
            # Move to the next step. 
            print(f'Rotation to target completed')
            step += 1

        return twist, step

