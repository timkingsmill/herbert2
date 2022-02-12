#!/usr/bin/env python

import math
from typing import Tuple

HALF_SQRT_3 = math.sqrt(3) / 2.0

# --------------------------------------------------------------------

def get_robot_position_from_wheel_position(a: float, b: float, c: float):
    """
    Given the a, b ad c wheel positions calculate
    the robot's x and y position.

    Robot Movement:
        +X: Forward
        -X: Reverse 
        +Y: Right
        -Y: Left

    """

    # Angle of wheel positions in radians.
    thetaA = math.radians( 60.0)   #60.0)
    thetaB = math.radians(180.0)   #0.0)
    thetaC = math.radians(300.0)   #-60.0)

    aX, aY = angle_to_vector(thetaA, a)
    bX, bY = angle_to_vector(thetaB, b)
    cX, cY = angle_to_vector(thetaC, c)

    # Add the x and y components of motor A, B and C
    x = (aX + bX + cX) / 3.0
    y = (aY + bY + cY) / 3.0

    return x, y

# --------------------------------------------------------------------

def wheel_velocity_from_vector(x, y):
    y = -y
    # calculate angle of the direction vector. (radians) 
    theta = math.atan2(x, y)

    # calculate magnitude of the direction vector.
    mag = math.sqrt((x * x) + (y * y)) 

    # calculate magnitude compensated x-component of the direction vector 
    vx = mag * math.cos(theta)
    # calculate magnitude compensated y-component of the direction vector
    vy = mag * math.sin(theta)

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

