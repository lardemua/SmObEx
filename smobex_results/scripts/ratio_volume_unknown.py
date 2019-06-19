#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3, Point
import numpy as np
from rospy.numpy_msg import numpy_msg

def getVolume(voxels):

    # rospy.loginfo("Received")

    total_volume = 0
    single_volume = 0
    one_zero = False

    for marker in voxels.markers:

        # points = np.array([marker.points.x, marker.points.y, marker.points.z])
        # print marker.action

        if marker.action == 0 and marker.ns == 'unknown_boxes':

            one_zero = True
            
            points = marker.points
            scale = marker.scale.x

            single_volume = scale ** 3
            total_volume += single_volume * len(points)
        
    if one_zero:
        
        print str(total_volume) + ' m3'
        return total_volume

    # rospy.signal_shutdown('Ended job')

if __name__ == '__main__':

    rospy.init_node('unknown_volume_ratio', anonymous=True)

    topic = '/cells_vis_array'

    one_take = False
    vol_it = 0
    vol_start = 0

    while not rospy.is_shutdown():

        if vol_start == 0:

            one_take = True
        
            # raw_input('Press Enter to get before volume')
        
            msg = rospy.wait_for_message(topic, MarkerArray)

            vol_start = getVolume(msg)    

            print 'Starting volume: ' + str(vol_start)   

        else:
            # raw_input('Press Enter to get after volume')

            msg = rospy.wait_for_message(topic, MarkerArray)

            vol_it = getVolume(msg)

            print 'Ratio unknown: ' + str(vol_it/vol_start)


