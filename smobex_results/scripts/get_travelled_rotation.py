#!/usr/bin/env python
import rospy
import tf
# import tf2_geometry_msgs, tf2_msgs, tf2_py, tf2_ros
import math
import numpy

# from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupActionResult
# from trajectory_msgs.msg import JointTrajectory
import industrial_msgs.msg


def isMoving():

    status = rospy.wait_for_message(
        '/robot_status', industrial_msgs.msg.RobotStatus)

    moving = status.in_motion.val == industrial_msgs.msg.TriState.TRUE

    return moving


if __name__ == '__main__':

    rospy.init_node('result_rotation_eef', anonymous=True)

    listener = tf.TransformListener()

    # total_dist = 0
    single_rot = 0
    # input1 = '...'
    trans2 = [0, 0, 0]
    first = True

    rate = rospy.Rate(1)

    print 'Rotation between poses, in rads'

    while not rospy.is_shutdown():

        try:
            (trans1, rot1) = listener.lookupTransform(
                '/camera_depth_optical_frame', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # raw_input('Enter...')
        # rate.sleep()

        in_motion = False

        while not in_motion:

            rospy.Rate(2).sleep()

            in_motion = isMoving()

            # print '1' + str(in_motion)

        while in_motion:

            rospy.Rate(2).sleep()

            in_motion = isMoving()

            # print '2' + str(in_motion)

        try:
            (trans2, rot2) = listener.lookupTransform(
                '/camera_depth_optical_frame', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        (r1, p1, y1) = tf.transformations.euler_from_quaternion(
            [rot1[0], rot1[1], rot1[2], rot1[3]])
        
        (r2, p2, y2) = tf.transformations.euler_from_quaternion(
            [rot2[0], rot2[1], rot2[2], rot2[3]])

        # d_r = abs(r2-r1)
        # d_p = abs(p2-p1)
        # d_y = abs(y2-y1)

        # print d_y

        # single_rot = d_r+d_p+d_y

        # print single_rot

        d_r = r2-r1
        d_p = p2-p1
        d_y = y2-y1

        # rotation_1 = tf.transformations.quaternion_matrix(rot1)
        # rotation_2 = tf.transformations.quaternion_matrix(rot2)

        # print tf.transformations.quaternion_from_euler(d_r,d_p,d_y)

        euler_matrix = tf.transformations.euler_matrix(d_r,d_p,d_y)        

        rotation = tf.transformations.rotation_from_matrix(euler_matrix)

        print abs(rotation[0])



