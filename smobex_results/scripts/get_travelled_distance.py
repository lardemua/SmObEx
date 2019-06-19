#!/usr/bin/env python
import rospy
import tf
import math

# from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupActionResult
# from trajectory_msgs.msg import JointTrajectory
import industrial_msgs.msg 

def isMoving():

    status = rospy.wait_for_message('/robot_status', industrial_msgs.msg.RobotStatus)
    
    moving = status.in_motion.val == industrial_msgs.msg.TriState.TRUE

    return moving


if __name__ == '__main__':

    rospy.init_node('result_distance_eef', anonymous=True)

    listener = tf.TransformListener()

    total_dist = 0
    single_dist = 0
    # input1 = '...'
    trans2 = [0,0,0]
    first = True

    rate = rospy.Rate(1)

    print 'Distance between poses, in m'

    while not rospy.is_shutdown():

        try:
            (trans1,rot1) = listener.lookupTransform('/camera_depth_optical_frame', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # raw_input('Enter...')
        # rate.sleep()

        in_motion = False

        while not in_motion:
            
            rospy.Rate(3).sleep()

            in_motion = isMoving()

            # print '1' + str(in_motion)

        while in_motion:

            rospy.Rate(3).sleep()

            in_motion = isMoving()

            # print '2' + str(in_motion)

        try:
            (trans2,rot2) = listener.lookupTransform('/camera_depth_optical_frame', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        single_dist = math.sqrt((trans2[0]-trans1[0])**2+(trans2[1]-trans1[1])**2+(trans2[2]-trans1[2])**2)
        total_dist += single_dist

        # print trans1
        # print trans2
        # print 'Single dist ' + str(i) + ': ' + str(single_dist) + ' m'
        # print 'Total dist: ' + str(total_dist) + ' m'
        # i+=1
        print single_dist

        # rate.sleep()

