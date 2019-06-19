#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupActionResult
from trajectory_msgs.msg import JointTrajectory

import industrial_msgs.msg 

def isMoving():

    status = rospy.wait_for_message('/robot_status', industrial_msgs.msg.RobotStatus)
    
    moving = status.in_motion.val == industrial_msgs.msg.TriState.TRUE

    return moving


def getVolume():

    # rospy.loginfo("Received")

    total_volume = 0
    single_volume = 0
    one_zero = False

    while not one_zero:

        voxels = rospy.wait_for_message('/cells_vis_array', MarkerArray)

        for marker in voxels.markers:

            # points = np.array([marker.points.x, marker.points.y, marker.points.z])
            # print marker.action

            if marker.action == 0 and marker.ns == 'unknown_boxes':

                one_zero = True

                points = marker.points
                scale = marker.scale.x

                single_volume = scale ** 3
                total_volume += single_volume * len(points)

    # print total_volume
    return total_volume

if __name__ == '__main__':

    rospy.init_node('unknown_volume_results', anonymous=True)

    # one_take = False
    vol_init = -1

    # i=0
    print 'Unknown volume, in m3'

    while not rospy.is_shutdown():   

        in_motion = False     

        while not in_motion:
            
            rospy.Rate(3).sleep()

            in_motion = isMoving()

            # print 1

        vol_start = getVolume()

        print vol_start

        in_motion = True

        while in_motion:
            
            rospy.Rate(3).sleep()

            in_motion = isMoving()

            # print 2
