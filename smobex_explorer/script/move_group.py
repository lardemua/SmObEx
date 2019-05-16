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

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

# joint_goal = [1.4254722595214844, 0.1137617751955986, -0.277296282351017, 0.9323690533638, -1.9833202362060547, 0.503192126750946]

# move_group.go(joint_goal, wait=True)
# move_group.stop()

# joint_goal = [1.4421101702212253, 0.32659461374066, -0.005513613440503273, 1.1074061843175271, -2.001746315270461, 0.6397495734833254]

# move_group.go(joint_goal, wait=True)
# move_group.stop(

joint_goal = [1.4587480809209663, 0.5394274522857214, 0.26626905547001045, 1.2824433152712542, -2.0201723943348675, 0.7763070202157049]

move_group.go(joint_goal, wait=True)
move_group.stop()

# joint_goal = [1.4753859916207073, 0.7522602908307828, 0.5380517243805241, 1.4574804462249813, -2.038598473399274, 0.9128644669480842]

# move_group.go(joint_goal, wait=True)
# move_group.stop()

# joint_goal = [1.4920239023204482, 0.9650931293758442, 0.8098343932910379, 1.6325175771787084, -2.0570245524636803, 1.0494219136804637]

# move_group.go(joint_goal, wait=True)
# move_group.stop()

# joint_goal = [1.508661813020189, 1.1779259679209058, 1.0816170622015515, 1.8075547081324355, -2.0754506315280867, 1.185979360412843]

# move_group.go(joint_goal, wait=True)
# move_group.stop()

# joint_goal = [1.52529972371993, 1.390758806465967, 1.3533997311120651, 1.9825918390861625, -2.093876710592493, 1.3225368071452224]


# move_group.go(joint_goal, wait=True)
# move_group.stop()

# joint_goal = [1.541937634419671, 1.6035916450110286, 1.625182400022579, 2.1576289700398896, -2.1123027896568995, 1.459094253877602]

# move_group.go(joint_goal, wait=True)
# move_group.stop()

joint_goal = [1.5585755451194119, 1.8164244835560899, 1.8969650689330928, 2.3326661009936167, -2.130728868721306, 1.5956517006099813]

move_group.go(joint_goal, wait=True)
move_group.stop()

# joint_goal = [1.5752134558191528, 2.0292573221011514, 2.1687477378436064, 2.507703231947344, -2.1491549477857124, 1.7322091473423606]

# move_group.go(joint_goal, wait=True)
# move_group.stop()
