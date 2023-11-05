#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    x_pos = data.pose.pose.position.x
    y_pos = data.pose.pose.position.y
    rospy.loginfo(rospy.get_caller_id() + "Robot position - x: {}, y: {}".format(x_pos, y_pos))
    with open('robot_pos.txt', 'w') as f:
        f.write('{}, {}\n'.format(x_pos, y_pos))


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()