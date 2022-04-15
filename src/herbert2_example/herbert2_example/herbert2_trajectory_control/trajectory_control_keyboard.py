#!/usr/bin/env python

import os
import sys
import time
import threading
from typing import Tuple
from copy import copy

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

import rclpy
from   rclpy.publisher import Publisher
from   rclpy.node import Node
from   rclpy.qos import QoSProfile
from   geometry_msgs.msg import Twist


# ............................................................................
# ............................................................................

class PublisherThread(threading.Thread):

    # ........................................................................

    def __init__(self, get_twist_msg, *args, **kwargs):
        super(PublisherThread, self).__init__(*args, **kwargs)
        self._stopper = threading.Event()
        self._get_twist_msg = get_twist_msg

        qos: QoSProfile = QoSProfile(depth=10)
        node: Node = rclpy.create_node('trajectory_control_keyboard')
        self._publisher: Publisher = node.create_publisher(Twist, 'cmd_twist_raw', qos)

    # ........................................................................

    def run(self):
        while self.stopped() is False:
            msg: Twist = self._get_twist_msg()
            self._publisher.publish(msg)
            time.sleep(0.01)

    # ........................................................................

    # function using _stopper event to stop the thread
    def stop(self):
        self._stopper.set()
 
    # ........................................................................

    def stopped(self):
        return self._stopper.isSet()

    # ........................................................................

# ............................................................................
# ............................................................................

terminal_msg = f"""
    Herbert2 Position Trajectory Control
    ---------------------------------------------------------

    Enter
        x: goal position x (unit: m)
        y: goal position y (unit: m)
        theta: goal orientation (range: -180 .. 180, unit: deg)
    """

# ............................................................................

def _get_target_pose() -> Tuple[float, float, float]:
    """ Print terminal message and get inputs """
    print(terminal_msg)

    input_x     = float(input("    Input x: "))
    input_y     = float(input("    Input y: "))
    input_theta = float(input("    Input theta: "))
    while input_theta > 179.0 or input_theta < -179.0:
        print("    Enter a value for theta between -180 and 180")
        input_theta = float(input("    Input theta: "))

    return input_x, input_y, input_theta

# ............................................................................
# ............................................................................

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    lock = threading.Lock()
    
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_theta: float = 0.0

    # Define funtion to be called by PublisherThread
    def get_twist_msg() -> Twist:
        lock.acquire()
        twist = Twist()
        try:
            twist.linear.x = pose_x
            twist.linear.y = pose_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = pose_theta
        finally:
            lock.release()
        return copy(twist)

    # Create a thread to publish Twist messages
    publisher_thread = PublisherThread(get_twist_msg)
    publisher_thread.daemon = True
    publisher_thread.name = 'publisher-thread'
    publisher_thread.start()

    try:
        while not publisher_thread.stopped():
            try:
                x, y, theta =_get_target_pose()
                lock.acquire()
                try:
                    pose_x = x
                    pose_y = y
                    pose_theta = theta
                finally:
                    lock.release()
            except KeyboardInterrupt:
                break
    finally:
        publisher_thread.stop()
        publisher_thread.join()




    """
        while(1) and (_publisher_thread_terminated is False):
        x, y, theta =_get_target_pose()

        twist = Twist()

        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = theta

        # Start publisher thread
        global _publisher_thread_running

        if (_publisher_thread_running is False) and (_publisher_thread_terminated is False):
            _publisher_thread_running = True
            publisher_thread.start()

        publisher.publish(twist)
    """

    print("Terminating Trajectory Keyboard Teleop")

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

# ............................................................................
# ............................................................................






















# ............................................................................
# ............................................................................
 

if __name__ == '__main__':
    main()

# ............................................................................
# ............................................................................
