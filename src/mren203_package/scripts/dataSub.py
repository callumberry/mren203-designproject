#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String #testing
import time

CO2 = 0
oldData = 10
x_pos = 0
y_pos = 0

def callback_int(data):
    global CO2 
    CO2 = data.data
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback_pose(data):
    global CO2
    global oldData
    x_pos = data.pose.pose.position.x
    y_pos = data.pose.pose.position.y
    if CO2 != 0 and x_pos != 0 and y_pos !=0:
        if oldData != CO2:
            rospy.loginfo(rospy.get_caller_id() + "Robot position - x: {}, y: {}".format(x_pos, y_pos))
            with open('/home/gonk/mren203_ws/src/mren203_package/scripts/robot_data.txt', 'a') as f:
                f.write('{} {} {}\n'.format(x_pos, y_pos, CO2))
            oldData = CO2
            CO2 = 0
            x_pos = 0
            y_pos = 0

#def callback_string(data):
#    global CO2
#    global oldData
#    string = data.data
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
# 
#    if CO2 != 0 and string != "":
#        if oldData != CO2:
#            rospy.loginfo(rospy.get_caller_id() + "I heard %s and %s", CO2, oldData)
#            with open('/home/gonk/mren203_ws/src/mren203_package/scripts/robot_data.txt', 'a') as f:
#                f.write('{} {}\n'.format(CO2, string))
#            oldData = CO2
#            CO2 = 0
#            string = ""
        

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("testInt", Int32, callback_int)

    #rospy.Subscriber("chatter", String, callback_string) #testing

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback_pose)
    rospy.spin()
if __name__ == '__main__':
    listener()