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
