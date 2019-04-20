#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <smobex_explorer/explorer.h>
#include <tf/transform_datatypes.h>

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

    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.6;
    // target_pose1.position.y = 0.3;
    // target_pose1.position.z = 0.6;
    // move_group.setPoseTarget(target_pose1);

    // geometry_msgs::PoseStamped target_pose1 = move_group.getRandomPose();

    // move_group.setPoseTarget(target_pose1);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    /* Uncomment below line when working with a real robot */
    // move_group.move();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped best_pose;
    float best_score = -1;

    for (size_t i = 0; i < 100; i++)
    {
        ros::Time t = ros::Time::now();

        geometry_msgs::PoseStamped target_pose1 = move_group.getRandomPose();

        move_group.setPoseTarget(target_pose1);

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        evaluatePose pose(20, 0.8, 10, 58 * M_PI / 180, 45 * M_PI / 180);

        tf::poseMsgToTF(target_pose1.pose, pose.view_pose);
        pose.evalPose();

        ros::Duration d = (ros::Time::now() - t);

        ROS_INFO_STREAM("Pose " << i << " score: " << pose.score);
        ROS_INFO_STREAM("The process took " << d.toSec() << " secs.");
        ROS_INFO("---------");

        if (pose.score > best_score) {
            best_score = pose.score;
            best_pose = target_pose1;
        }
        
    }

    ROS_INFO_STREAM("Best score was " << best_score);
    move_group.setPoseTarget(best_pose);
    move_group.plan(my_plan);
    move_group.move();

    ros::shutdown();

    return 0;
}
