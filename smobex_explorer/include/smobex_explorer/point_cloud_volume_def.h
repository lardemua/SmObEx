#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

void filterBox(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    ros::NodeHandle n;

    ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    geometry_msgs::Point p;

    ros::Time t = ros::Time::now();

    std::string _fixed_frame_id = "/world";

    float _x_min = 0, _x_max = 1;
    float _y_min = 0, _y_max = 1;
    float _z_min = 0, _z_max = 1;

    ros::param::get("~" + ros::names::remap("x_max"), _x_max);
    ros::param::get("~" + ros::names::remap("x_min"), _x_min);

    ros::param::get("~" + ros::names::remap("y_max"), _y_max);
    ros::param::get("~" + ros::names::remap("y_min"), _y_min);

    ros::param::get("~" + ros::names::remap("z_max"), _z_max);
    ros::param::get("~" + ros::names::remap("z_min"), _z_min);

    ros::param::get("~fixed_frame", _fixed_frame_id);

    float scale = 1;
    ros::param::get("~scale", scale);

    float start_center_x = (_x_max + _x_min) / 2;
    float start_center_y = (_y_max + _y_min) / 2;
    float start_center_z = (_z_max + _z_min) / 2;

    _x_max *= scale;
    _y_max *= scale;
    _z_max *= scale;

    _x_min *= scale;
    _y_min *= scale;
    _z_min *= scale;

    float end_center_x = (_x_max + _x_min) / 2;
    float end_center_y = (_y_max + _y_min) / 2;
    float end_center_z = (_z_max + _z_min) / 2;

    _x_max = _x_max - (end_center_x - start_center_x);
    _y_max = _y_max - (end_center_y - start_center_y);
    _z_max = _z_max - (end_center_z - start_center_z);

    _x_min = _x_min - (end_center_x - start_center_x);
    _y_min = _y_min - (end_center_y - start_center_y);
    _z_min = _z_min - (end_center_z - start_center_z);

    //filter cloud

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA> out_cloud;
    pcl::fromROSMsg(*cloud_msg, *in_cloud);
    pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(_x_min, _y_min, _z_min, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_x_max, _y_max, _z_max, 1.0));
    boxFilter.setInputCloud(in_cloud);
    boxFilter.filter(out_cloud);

    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(out_cloud, out_cloud_msg);

    out_cloud_msg.header.frame_id = _fixed_frame_id;
    out_cloud_msg.header.stamp = t;

    pub_cloud.publish(out_cloud_msg);

    ros::spinOnce();
    
}

visualization_msgs::MarkerArray markerBox()
{
    ros::NodeHandle n;

    ros::Publisher pub_box = n.advertise<visualization_msgs::MarkerArray>("boxLines_array", 10);

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    geometry_msgs::Point p;

    ros::Time t = ros::Time::now();

    std::string _fixed_frame_id = "/world";

    float _x_min = 0, _x_max = 1;
    float _y_min = 0, _y_max = 1;
    float _z_min = 0, _z_max = 1;

    ros::param::get("~" + ros::names::remap("x_max"), _x_max);
    ros::param::get("~" + ros::names::remap("x_min"), _x_min);

    ros::param::get("~" + ros::names::remap("y_max"), _y_max);
    ros::param::get("~" + ros::names::remap("y_min"), _y_min);

    ros::param::get("~" + ros::names::remap("z_max"), _z_max);
    ros::param::get("~" + ros::names::remap("z_min"), _z_min);

    ros::param::get("~fixed_frame", _fixed_frame_id);

    float scale = 1;
    ros::param::get("~scale", scale);

    float start_center_x = (_x_max + _x_min) / 2;
    float start_center_y = (_y_max + _y_min) / 2;
    float start_center_z = (_z_max + _z_min) / 2;

    _x_max *= scale;
    _y_max *= scale;
    _z_max *= scale;

    _x_min *= scale;
    _y_min *= scale;
    _z_min *= scale;

    float end_center_x = (_x_max + _x_min) / 2;
    float end_center_y = (_y_max + _y_min) / 2;
    float end_center_z = (_z_max + _z_min) / 2;

    _x_max = _x_max - (end_center_x - start_center_x);
    _y_max = _y_max - (end_center_y - start_center_y);
    _z_max = _z_max - (end_center_z - start_center_z);

    _x_min = _x_min - (end_center_x - start_center_x);
    _y_min = _y_min - (end_center_y - start_center_y);
    _z_min = _z_min - (end_center_z - start_center_z);

    marker.header.frame_id = _fixed_frame_id;
    marker.header.stamp = t;
    marker.ns = "wireframe";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);

    marker.pose.position.x = (_x_min + _x_max) / 2;
    marker.pose.position.y = (_y_min + _y_max) / 2;
    marker.pose.position.z = (_z_min + _z_max) / 2;

    marker.scale.x = 0.01; // width of the line

    double x = fabs(_x_min - marker.pose.position.x);
    double y = fabs(_y_min - marker.pose.position.y);
    double z = fabs(_z_min - marker.pose.position.z);

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);

    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);

    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);

    marker_array.markers.push_back(marker);

    return marker_array;
}