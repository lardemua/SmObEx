#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorer_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

#if 1

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.6;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.6;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

#else

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = 0.0; // radians
    joint_group_positions[1] = 0.0; // radians
    joint_group_positions[2] = 0.0; // radians
    joint_group_positions[3] = 0.0; // radians
    joint_group_positions[4] = 0.0; // radians
    joint_group_positions[5] = 0.0; // radians
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

#endif

    /* Uncomment below line when working with a real robot */
    move_group.move();

    ros::shutdown();

    return 0;
}
