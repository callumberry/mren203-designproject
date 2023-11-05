#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import signal

def move_forward():
    def signal_handler(sig, frame):
        stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        pub.publish(twist)
        rate.sleep()

def move_backward():
    def signal_handler(sig, frame):
        stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        pub.publish(twist)
        rate.sleep()

def turn_clockwise():
    def signal_handler(sig, frame):
        stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        pub.publish(twist)
        rate.sleep()

def turn_counterclockwise():
    def signal_handler(sig, frame):
        stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        pub.publish(twist)
        rate.sleep()

def stop():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('move_rovot', anonymous=True)
    if len(sys.argv) > 1:
        function_name = sys.argv[1]
        if function_name == 'move_forward':
            move_forward()
        elif function_name == 'move_backward':
            move_backward()
        elif function_name == 'turn_clockwise':
            turn_clockwise()
        elif function_name == 'turn_counterclockwise':
            turn_counterclockwise()
        elif function_name == 'stop':
            stop()
    else:
        print("Usage: {} [function_name]".format(sys.argv[0]))