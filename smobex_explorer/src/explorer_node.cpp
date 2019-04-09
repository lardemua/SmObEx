#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <smobex_explorer/explorer.h>

using namespace std;

geometry_msgs::PoseArray genMultiPose(pcl::PointCloud<pcl::PointXYZ> center_points)
{
    std::string fixed_frame = "/map";
    float r_min = 0, r_max = 1;
    int n_poses = 20;

    ros::param::get("~fixed_frame", fixed_frame);
    ros::param::get("~r_min", r_min);
    ros::param::get("~r_max", r_max);
    ros::param::get("~n_poses", n_poses);

    geometry_msgs::PoseArray point_poses;

    point_poses.header.frame_id = fixed_frame;
    point_poses.header.stamp = ros::Time::now();

    int num_of_points = center_points.size();

    evaluatePose poses[num_of_points];

    // ROS_INFO("Num of points: %d", num_of_points);
    // ROS_INFO("Num of poses: %d", n_poses);

    for (int i = 0; i < num_of_points; i++)
    {
        for (int n = 0; n < n_poses; n++)
        {
            pcl::PointXYZ pcl_point = center_points.at(i);
            tf::Point tf_point;
            tf::Pose tf_pose;
            geometry_msgs::Pose one_pose;

            tf_point.setX(pcl_point.x);
            tf_point.setY(pcl_point.y);
            tf_point.setZ(pcl_point.z);

            // tf_pose = genPose(r_min, r_max, tf_point);
            poses[i].genPose(r_min, r_max, tf_point);
            poses[i].evalPose();

            tf_pose = poses[i].view_pose;

            tf::poseTFToMsg(tf_pose, one_pose);

            point_poses.poses.push_back(one_pose);

            // ROS_INFO("Pose published: %d", n);
        }
    }

    return point_poses;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorer_node");

    ros::NodeHandle n;

    ros::Publisher pub_poseArray = n.advertise<geometry_msgs::PoseArray>("/pose_array", 10);

    // ros::Subscriber sub = n.subscribe("/octomap_full", 1, octomapCallback);

    // ros::Subscriber sub_spaceDefined = n.subscribe<visualization_msgs::MarkerArray>("/cells_vis_array", 10, mapCallback);
    // ros::Subscriber octomap_sub = n.subscribe("octomap_full", 10, &evaluatePose::writeOctomap, &poseee);

    ros::Rate loop_rate(10);

    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ> test;

    test.push_back(pcl::PointXYZ(1.5, 0, 0.5));

    geometry_msgs::PoseArray all_poses;
    all_poses = genMultiPose(test);

    while (ros::ok())
    {
        pub_poseArray.publish(all_poses);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
