#!/usr/bin/env python
import rospy
import tf
import math

if __name__ == '__main__':

    rospy.init_node('result_distance_eef', anonymous=True)

    listener = tf.TransformListener()

    total_dist = 0
    single_dist = 0
    input1 = '...'
    trans2 = [0,0,0]

    while not rospy.is_shutdown():

        if not input1 == '':
            
            input1 = raw_input('Press Enter to record point Start')

            try:
                (trans1,rot1) = listener.lookupTransform('/camera_depth_optical_frame', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        else:   
            trans1 = trans2         

        input2 = raw_input('Press Enter to record point End')

        try:
            (trans2,rot2) = listener.lookupTransform('/camera_depth_optical_frame', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        single_dist = math.sqrt((trans2[0]-trans1[0])**2+(trans2[1]-trans1[1])**2+(trans2[2]-trans1[2])**2)
        total_dist += single_dist

        print trans1
        print trans2
        print 'Single dist: ' + str(single_dist) + ' m'
        print 'Total dist: ' + str(total_dist) + ' m'

