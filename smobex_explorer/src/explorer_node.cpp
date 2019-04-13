#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <smobex_explorer/explorer.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorer_node");

    ros::NodeHandle n;

    ros::Publisher pub_poseArray = n.advertise<geometry_msgs::PoseArray>("/pose_array", 10);
    ros::Publisher pub_lines = n.advertise<visualization_msgs::Marker>("/ray_cast_lines", 10);

    ros::Publisher pub_blue = n.advertise<visualization_msgs::MarkerArray>("/discovered_space", 10);

    // ros::Subscriber sub = n.subscribe("/octomap_full", 1, octomapCallback);

    // ros::Subscriber sub_spaceDefined = n.subscribe<visualization_msgs::MarkerArray>("/cells_vis_array", 10, mapCallback);
    // ros::Subscriber octomap_sub = n.subscribe("octomap_full", 10, &evaluatePose::writeOctomap, &poseee);

    ros::Rate loop_rate(10);

    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ> center_points;

    center_points.push_back(pcl::PointXYZ(1.5, 0, 0.5));

    std::string fixed_frame = "/map";
    float r_min = 0, r_max = 1;
    int n_poses = 20;

    ros::param::get("~fixed_frame", fixed_frame);
    ros::param::get("~r_min", r_min);
    ros::param::get("~r_max", r_max);
    ros::param::get("~n_poses", n_poses);

    geometry_msgs::PoseArray point_poses;
    visualization_msgs::Marker line_vis;

    ros::Time t = ros::Time::now();

    point_poses.header.frame_id = fixed_frame;
    point_poses.header.stamp = t;

    line_vis.header.frame_id = fixed_frame;
    line_vis.header.stamp = t;
    // line_vis.ns = "rays";
    line_vis.action = visualization_msgs::Marker::ADD;
    line_vis.pose.orientation.w = 1.0;
    line_vis.id = 0;
    line_vis.type = visualization_msgs::Marker::LINE_LIST;
    line_vis.scale.x = 0.001;

    line_vis.color.r = 0.5;
    line_vis.color.g = 0.5;
    line_vis.color.b = 0.5;
    line_vis.color.a = 1.0;

    int num_of_points = center_points.size();

    // evaluatePose poses[n_poses];
    std::vector<evaluatePose> poses(n_poses, evaluatePose(20, 0.8, 58 * M_PI / 180, 45 * M_PI / 180));

    int k = 0;
    visualization_msgs::MarkerArray found_space;
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
            poses[k].genPose(r_min, r_max, tf_point);

            tf_pose = poses[k].view_pose;

            tf::poseTFToMsg(tf_pose, one_pose);

            point_poses.poses.push_back(one_pose);

            k++;
            // ROS_INFO("Pose published: %d", n);
        }

        k = 0;
        for (int i = 0; i < num_of_points; i++)
        {
            for (int n = 0; n < n_poses; n++)
            {
                poses[k].evalPose();

                octomap::point3d_list ray_points = poses[k].ray_points_list;

                for (octomap::point3d_list::iterator it = ray_points.begin(); it != ray_points.end(); it++)
                {
                    geometry_msgs::Point point;
                    point.x = it->octomath::Vector3::x();
                    point.y = it->octomath::Vector3::y();
                    point.z = it->octomath::Vector3::z();

                    line_vis.points.push_back(point);
                }
                k++;
            }
        }

        found_space = poses[0].discoveredBoxes(fixed_frame);
    }

    while (ros::ok())
    {
        pub_poseArray.publish(point_poses);
        pub_lines.publish(line_vis);
        pub_blue.publish(found_space);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
