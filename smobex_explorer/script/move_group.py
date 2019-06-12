#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # move_group.set_planner_id("RRTConnectkConfigDefault")

    # box_name = 'box'
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "base_link"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 0.2
    # box_pose.pose.position.x = 0.6
    # # scene.add_box(box_name, box_pose, size=(0.4, 0.4, 0.4))
    # scene.attach_box('base_link', box_name, box_pose, size = (1, 1, 1))
    
    # rospy.sleep(10)

    # print scene.get_objects()

    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # # joint_goal = [1.4587480809209663, 0.5394274522857214, 0.26626905547001045, 1.2824433152712542, -2.0201723943348675, 0.7763070202157049]

    # # move_group.go(joint_goal, wait=True)
    # # move_group.stop()

    joint_goal = [1.5585755451194119, 1.8164244835560899, 1.8969650689330928, 2.3326661009936167, -2.130728868721306, 1.5956517006099813]

    move_group.go(joint_goal, wait=True)
    move_group.stop()
