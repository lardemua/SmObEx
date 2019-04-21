#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <smobex_explorer/explorer.h>

#include <tf/transform_datatypes.h>

// #include <octomap/octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap_ros/conversions.h>

using namespace std;

// void lol(const )

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorer_node");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    evaluatePose pose_test(20, 0.8, 10, 58 * M_PI / 180, 45 * M_PI / 180);

    // ros::Subscriber octomapFull_sub = n.subscribe("/octomap_full", 1, &evaluatePose::writeKnownOctomapCallback, &pose);
    // ros::Subscriber unknownFullMap_sub = n.subscribe("/unknown_full_map", 1, &evaluatePose::writeUnknownOctomapCallback, &pose);

    pose_test.writeKnownOctomap();
    pose_test.writeUnknownOctomap();
    move_group.setPlanningTime(0.3);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped target_pose, best_pose;
    float best_score = -1;
    int best_pose_n = -1;

    for (size_t i = 0; i < 100; i++)
    {
        ros::Time t = ros::Time::now();

        target_pose = move_group.getRandomPose(move_group.getEndEffectorLink().c_str());

        // ROS_INFO_STREAM("Target pose: " << target_pose);

        move_group.setPoseTarget(target_pose, move_group.getEndEffectorLink().c_str());

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ros::Duration d = (ros::Time::now() - t);
        ROS_WARN_STREAM("Planning took " << d.toSec() << " secs.");

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        if (success)
        {
            tf::poseMsgToTF(target_pose.pose, pose_test.view_pose);

            t = ros::Time::now();
            pose_test.evalPose();
            d = (ros::Time::now() - t);
            ROS_WARN_STREAM("Evaluation took " << d.toSec() << " secs.");

            ROS_INFO_STREAM("Pose " << i << " score: " << pose_test.score);

            if (pose_test.score > best_score)
            {
                best_pose_n = i;
                best_score = pose_test.score;
                best_pose = target_pose;
                ROS_INFO_STREAM("Best pose now: " << best_pose);
                ROS_INFO_STREAM("Target pose now: " << target_pose);
            }
        }

        ROS_INFO("---------");
    }

    ROS_INFO_STREAM("Best score was " << best_score << " of pose " << best_pose_n);
    ROS_INFO_STREAM("Best pose: " << best_pose);
    move_group.setPoseTarget(best_pose);
    move_group.setPlanningTime(1);
    move_group.plan(my_plan);
    move_group.move();

    ros::shutdown();

    return 0;
}
