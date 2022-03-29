#!/usr/bin/env python

import math
from typing import Tuple

HALF_SQRT_3 = math.sqrt(3) / 2.0

# --------------------------------------------------------------------

def wheel_velocity_from_vector(x, y):
    """
        +y : move forward
        +x : move right
    """

    # calculate angle of the direction vector. (radians) 
    theta = math.atan2(x, y)

    # calculate magnitude of the direction vector.
    mag = 1.0 ## math.sqrt((x * x) + (y * y)) 

    # calculate magnitude compensated x-component of the direction vector 
    #vx = mag * math.cos(theta)
    vx = mag * math.sin(theta)
    # calculate magnitude compensated y-component of the direction vector
    #vy = mag * math.sin(theta)
    vy = mag * math.cos(theta)

    # calculate axis velocity 
    axisA_vel = 0.5 * vx - (HALF_SQRT_3 * vy)
    axisB_vel = -vx
    axisC_vel = 0.5 * vx + (HALF_SQRT_3 * vy)

    return [axisA_vel, axisB_vel, axisC_vel]

# --------------------------------------------------------------------

def angle_to_vector(theta, magnitude):
    # theta in radians
    # calculate x-component of the direction vector ( foward / back )
    vx = math.sin(theta) * magnitude
    # calculate y-component of the direction vector (right / left )
    vy = math.cos(theta) * magnitude
    return [vx, vy]

# --------------------------------------------------------------------

def calc_vector_magnitude(x, y):
    # calculate magnitude of the direction vector.
    mag = math.sqrt((x * x) + (y * y)) 
    return mag

# --------------------------------------------------------------------

def vector_to_angle(x, y):
    # calculate angle of the direction vector. (radians) 
    theta = math.atan2(x, y)
    return theta

# --------------------------------------------------------------------

def vector_to_polar(x: float, y: float) -> Tuple[float, float]:
    # calculate angle of the direction vector. (radians) 
    theta = math.atan2(x, y)
    # calculate magnitude of the direction vector.
    mag = math.sqrt((x * x) + (y * y)) 
    return theta, mag

# --------------------------------------------------------------------

