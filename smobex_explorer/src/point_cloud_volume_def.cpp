#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/filters/crop_box.h>
#include <pcl/point_types.h>

#include <smobex_explorer/point_cloud_volume_def.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "volume_box");

    ros::NodeHandle nh;

    ros::Publisher pub_box = nh.advertise<visualization_msgs::MarkerArray>("boxLines_array", 10);

    visualization_msgs::MarkerArray box = markerBox();

    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, filterBox);

    while (ros::ok())
    {
        pub_box.publish(box);

        ros::spinOnce();

        ros::Duration(0.5).sleep();
    }

    return 0;
}
