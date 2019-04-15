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
    ros::Publisher pub_lines = n.advertise<visualization_msgs::MarkerArray>("/ray_cast_lines", 10);

    ros::Publisher pub_blue = n.advertise<visualization_msgs::MarkerArray>("/discovered_space", 10);
    ros::Publisher pub_text = n.advertise<visualization_msgs::MarkerArray>("/pose_text", 10);

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

    ros::Time t = ros::Time::now();

    geometry_msgs::PoseArray point_poses;

    point_poses.header.frame_id = fixed_frame;
    point_poses.header.stamp = t;

    int num_of_points = center_points.size();

    std::vector<evaluatePose> poses(n_poses, evaluatePose(20, 0.8, 58 * M_PI / 180, 45 * M_PI / 180));

    int k = 0;

    visualization_msgs::Marker text;
    visualization_msgs::MarkerArray found_space, text_pose, line_vis;

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

            int score = poses[k].getScore();

            text.header.frame_id = fixed_frame;
            text.header.stamp = t;

            text.id = k;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;

            text.scale.z = 0.05;
            text.color.r = 0.0;
            text.color.g = 0.0;
            text.color.b = 0.0;
            text.color.a = 1.0;
            text.pose.position.x = one_pose.position.x - 0.1;
            text.pose.position.y = one_pose.position.y - 0.1;
            text.pose.position.z = one_pose.position.z - 0.1;
            text.pose.orientation.x = 0.0;
            text.pose.orientation.y = 0.0;
            text.pose.orientation.z = 0.0;
            text.pose.orientation.w = 1.0;

            text.text = "Point: " + to_string(i) + "\n" + "Pose: " + to_string(k) + "\n" + "Score: " + to_string(score);
            // text.ns = ns;

            text_pose.markers.push_back(text);

            visualization_msgs::Marker line;
            line = poses[k].rayLinesVis(fixed_frame, k);

            line_vis.markers.push_back(line);

            visualization_msgs::MarkerArray single_view_boxes;
            single_view_boxes = poses[k].discoveredBoxesVis(fixed_frame, k);

            for (size_t i = 0; i < single_view_boxes.markers.size(); i++)
            {
                found_space.markers.push_back(single_view_boxes.markers[i]);
            }

            k++;
            // ROS_INFO("Pose published: %d", n);
        }
    }

    // line_vis = poses[0].rayLinesVis(fixed_frame);
    // found_space = poses[0].discoveredBoxesVis(fixed_frame, 0);

    while (ros::ok())
    {
        pub_poseArray.publish(point_poses);
        pub_lines.publish(line_vis);
        pub_blue.publish(found_space);
        pub_text.publish(text_pose);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
