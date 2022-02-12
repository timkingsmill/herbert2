#!/usr/bin/env python

import os
import select
import sys
from typing import cast

from geometry_msgs.msg import Twist

import rclpy
from   rclpy.client import Client
from   rclpy.publisher import Publisher
from   rclpy.node import Node
from   rclpy.qos import QoSProfile

from herbert2_odrive_interfaces.srv import CalibrateODriveService

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 1.0 #0.22
LIN_VEL_STEP_SIZE = 0.02 #0.01

HERBERT2_MAX_ANG_VEL = 2.84

ANG_VEL_STEP_SIZE = 0.1

msg = """
        Scalar Robot Movement:
            +X: Forward
            -X: Reverse 
            +Y: Right
            -Y: Left

Control Your Herbert2 robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# -----------------------------------------------------------------------------

def print_linear_vel(x, y, z):
    print(f'Velocity x: {x:.3f} y: {y:.3f} z: {z:.3f}')

# -----------------------------------------------------------------------------

def make_simple_profile(output, input, slope):
    if input > output:
        output = min(input, output + slope)
    elif input < output:
        output = max(input, output - slope)
    else:
        output = input

    return output

# -----------------------------------------------------------------------------

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

# -----------------------------------------------------------------------------

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

# -----------------------------------------------------------------------------

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -HERBERT2_MAX_ANG_VEL, HERBERT2_MAX_ANG_VEL)

# -----------------------------------------------------------------------------

def calibrate_odrives(node: Node) -> None:
    print('Calibrate ODrives')


    request = CalibrateODriveService.Request()
    future = _cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(future.result().message)

# -----------------------------------------------------------------------------

#def stop
# -----------------------------------------------------------------------------

def publish_velocity(velocity: Twist) -> None:
    _pub.publish(velocity)

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')

    global _pub
    #_pub = cast(Publisher, node.create_publisher(Twist, 'cmd_vel_raw', qos))
    _pub = cast(Publisher, node.create_publisher(Twist, 'cmd_vel', qos))
    
    global _cli
    _cli = cast(Client, node.create_client(CalibrateODriveService, 'calibrate_odrives'))

    status = 0

    target_x_linear_velocity = 0.0
    target_y_linear_velocity = 0.0
    target_z_linear_velocity = 0.0

    control_x_linear_velocity = 0.0
    control_y_linear_velocity = 0.0
    control_z_linear_velocity = 0.0

    target_angular_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_x_linear_velocity =\
                    check_linear_limit_velocity(target_x_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_linear_vel(target_x_linear_velocity, target_y_linear_velocity, target_z_linear_velocity)

            elif key == 'x':
                target_x_linear_velocity =\
                    check_linear_limit_velocity(target_x_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_linear_vel(target_x_linear_velocity, target_y_linear_velocity, target_z_linear_velocity)

            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_x_linear_velocity, target_angular_velocity)

                target_y_linear_velocity =\
                    check_linear_limit_velocity(target_y_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_linear_vel(target_x_linear_velocity, target_y_linear_velocity, target_z_linear_velocity)

            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                #print_vels(target_x_linear_velocity, target_angular_velocity)

                target_y_linear_velocity =\
                    check_linear_limit_velocity(target_y_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_linear_vel(target_x_linear_velocity, target_y_linear_velocity, target_z_linear_velocity)

            elif key == 'c':
                """ Calibrate ODrives """
                calibrate_odrives(node)

            elif key == ' ' or key == 's':
                target_x_linear_velocity = 0.0
                control_x_linear_velocity = 0.0

                target_y_linear_velocity = 0.00
                control_y_linear_velocity = 0.0

                target_angular_velocity = 0.0
                control_angular_velocity = 0.0

                print_linear_vel(target_x_linear_velocity, target_y_linear_velocity, target_z_linear_velocity)
            
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_x_linear_velocity = make_simple_profile(
                control_x_linear_velocity,
                target_x_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_y_linear_velocity = make_simple_profile(
                control_y_linear_velocity,
                target_y_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_x_linear_velocity
            twist.linear.y = control_y_linear_velocity
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            publish_velocity(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        publish_velocity(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
