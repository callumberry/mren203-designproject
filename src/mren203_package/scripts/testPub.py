#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32

int = 0

def testInt():
    pub = rospy.Publisher('testInt', Int32, queue_size=10)
    rospy.init_node('testInt', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        global int 
        int = int + 1
        rospy.loginfo(int)
        pub.publish(int)
        rate.sleep()

if __name__ == '__main__':
    try:
        testInt()
    except rospy.ROSInterruptException:
        pass