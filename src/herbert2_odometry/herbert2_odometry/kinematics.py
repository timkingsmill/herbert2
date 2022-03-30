import math
from typing import Tuple

sqrt3: float = math.sqrt(3) 

# ..............................................................................................

def wheel_frame_to_robot_frame(left: float, rear: float, right: float) -> Tuple[float, float]:
    # Forward kinematics
    x: float = -(((2 * rear) - left - right) / 3.0)
    y: float = ((sqrt3 * left) - (sqrt3 * right)) / 3.0

    return x, y

# ..............................................................................................

def robot_frame_to_wheel_frame(x: float, y: float) -> Tuple[float, float, float]:
    # Reverse kinematics
    left: float  = (-1.0 * (x / 2.0)) - ((sqrt3 * y) / 2.0)
    rear: float  = x
    right: float = (-1.0 * (x / 2.0)) + ((sqrt3 * y) / 2.0)

    return left, rear, right

# ..............................................................................................

def robot_frame_to_wheel_frame_velocity(x: float, y: float) -> Tuple[float, float, float]:
    # Reverse kinematics

    left: float = 0.0
    rear: float = 0.0
    right: float = 0.0
    
    # Calculate and normalize the direction vector.
    mag = math.sqrt((x * x) + (y * y)) 
    if (mag > 0.000001):
        vx = x / mag
        vy = y / mag

        left  = (-1.0 * (vx / 2.0)) - ((sqrt3 * vy) / 2.0)
        rear  = vx
        right = (-1.0 * (vx / 2.0)) + ((sqrt3 * vy) / 2.0)

    return left, rear, right

# ..............................................................................................
