#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import JointState


def callback(data):
    joint1 = data.position[0]
    joint2 = data.position[1]
    joint3 = data.position[2]
    joint4 = data.position[3]
    joint5 = data.position[4]
    joint6 = data.position[5]

    if joint1 <= -2.967 or joint1 >= 2.967:
        rospy.logerr("Joint 1 out of bounds" + str(joint1))

    if joint2 <= -1.570 or joint2 >= 2.792:
        rospy.logerr("Joint 2 out of bounds" + str(joint2))

    if joint3 <= -2.443 or joint3 >= 2.967:
        rospy.logerr("Joint 3 out of bounds" + str(joint3))

    if joint4 <= -3.316 or joint4 >= 3.316:
        rospy.logerr("Joint 4 out of bounds" + str(joint4))

    if joint5 <= -2.443 or joint5 >= 2.443:
        rospy.logerr("Joint 5 out of bounds" + str(joint5))

    if joint6 <= -6.283 or joint6 >= 6.283:
        rospy.logerr("Joint 6 out of bounds" + str(joint6))


def listener():

    rospy.init_node('joint_check', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
