#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <smobex_explorer/explorer.h>

#include <tf/tf.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <octomap/octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap_ros/conversions.h>

using namespace std;

geometry_msgs::Quaternion getOrientation(geometry_msgs::PoseStamped pose)
{
    tf::Vector3 z_direction, y_direction, x_direction, rand_vector;
    tf::Matrix3x3 rotation_matrix;
    tf::Quaternion view_orientation;

    geometry_msgs::Quaternion quat_out;

    std::vector<geometry_msgs::Point> centers;

    float xc = 1;
    float yc = 0;
    float zc = -0.2;

    float x = pose.pose.position.x;
    float y = pose.pose.position.y;
    float z = pose.pose.position.z;

    z_direction.setX(xc - x);
    z_direction.setY(yc - y);
    z_direction.setZ(zc - z);

    rand_vector.setX((double)rand() / RAND_MAX);
    rand_vector.setY((double)rand() / RAND_MAX);
    rand_vector.setZ((double)rand() / RAND_MAX);

    y_direction = z_direction.cross(rand_vector);
    y_direction.setZ(-1 * abs(y_direction.getZ()));
    y_direction.normalize();

    x_direction = y_direction.cross(z_direction);
    x_direction.normalize();

    rotation_matrix.setValue(x_direction.getX(), y_direction.getX(), z_direction.getX(), x_direction.getY(),
                             y_direction.getY(), z_direction.getY(), x_direction.getZ(), y_direction.getZ(),
                             z_direction.getZ());

    rotation_matrix.getRotation(view_orientation);
    view_orientation.normalize();

    tf::quaternionTFToMsg(view_orientation, quat_out);

    return quat_out;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorer_node");

    ros::NodeHandle n;

    ros::Publisher pub_arrows = n.advertise<visualization_msgs::MarkerArray>("/pose_arrows", 10);

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

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped target_pose, best_pose;
    visualization_msgs::MarkerArray all_poses;

    pose_test.writeKnownOctomap();
    pose_test.writeUnknownOctomap();
    move_group.setPlanningTime(0.3);

    float best_score = -1;
    int best_pose_n = -1;

    for (size_t i = 0; i < 100; i++)
    {

        srand(time(NULL));

        target_pose = move_group.getRandomPose();
        // move_group.setPositionTarget(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        geometry_msgs::Quaternion quat_orient = getOrientation(target_pose);
        // move_group.setOrientationTarget(quat_orient.x, quat_orient.y, quat_orient.z, quat_orient.w);
        target_pose.pose.orientation = quat_orient;

        move_group.setPoseTarget(target_pose);

        ros::Time t = ros::Time::now();

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

            visualization_msgs::Marker arrow;

            arrow.header.stamp = ros::Time::now();
            arrow.header.frame_id = "/base_link";

            arrow.id = i;

            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.color = pose_test.score_color;

            arrow.pose.position = target_pose.pose.position;
            // arrow.pose.orientation = target_pose.pose.orientation; //TODO fix orientation

            tf::Quaternion q_rot, q_new;
            geometry_msgs::Quaternion q_arrow;
            tf::quaternionMsgToTF(target_pose.pose.orientation, q_new);

            q_rot.setRPY(0, -M_PI / 2, 0);
            q_new = q_new * q_rot;
            q_new.normalize();

            tf::quaternionTFToMsg(q_new, q_arrow);
            arrow.pose.orientation = q_arrow;

            arrow.scale.x = 0.1;
            arrow.scale.y = 0.05;
            arrow.scale.z = 0.05;

            all_poses.markers.push_back(arrow);

            pub_arrows.publish(all_poses);
        }

        ROS_INFO("---------");
    }

    ROS_INFO_STREAM("Best score was " << best_score << " of pose " << best_pose_n);
    ROS_INFO_STREAM("Best pose: " << best_pose);
    move_group.setPoseTarget(best_pose);
    move_group.setPlanningTime(1);
    move_group.plan(my_plan);

    ROS_INFO_STREAM("Go to pose?");
    int c = getchar();

    if (c == 'y' || c == 'Y')
    {
        // move_group.move();
        move_group.execute(my_plan);
    }

    // ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
