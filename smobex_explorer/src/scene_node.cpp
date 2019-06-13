#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle n;

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    float robot_reach = 0.951;
    ros::param::get("~robot_reach",robot_reach);

    std::vector<geometry_msgs::Pose> wall_poses;
    geometry_msgs::Pose box_pose;

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = robot_reach;
    box_pose.position.y = 0;
    box_pose.position.z = 0;
    wall_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -robot_reach;
    box_pose.position.y = 0;
    box_pose.position.z = 0;
    wall_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.7071068;
    box_pose.orientation.w = 0.7071068;
    box_pose.position.x = 0;
    box_pose.position.y = robot_reach;
    box_pose.position.z = 0;
    wall_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.7071068;
    box_pose.orientation.w = 0.7071068;
    box_pose.position.x = 0;
    box_pose.position.y = -robot_reach;
    box_pose.position.z = 0;
    wall_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.7071068;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.7071068;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = robot_reach;
    wall_poses.push_back(box_pose);

    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.7071068;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.7071068;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = -robot_reach;
    wall_poses.push_back(box_pose);

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.005;
    primitive.dimensions[1] = 3;
    primitive.dimensions[2] = 3;

    for (size_t i = 0; i < wall_poses.size(); i++)
    {
        collision_object.id = "box" + std::to_string(i);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(wall_poses[i]);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }

    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::waitForShutdown();

    return 0;
}