#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <octomap/math/Quaternion.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

using namespace std;


tf::Pose genPose(float r_min, float r_max, tf::Point observation_center)
{
    tf::Pose view_pose;
    tf::Point view_origin;
    tf::Vector3 z_direction, y_direction, x_direction, rand_vector;
    tf::Matrix3x3 rotation_matrix;
    tf::Quaternion view_orientation;

    float theta, psi, radius;

    if (r_max < 0 || r_min < 0)
    {
        ROS_WARN("r_max/r_min should be positive!");
        r_min = abs(r_min);
        r_max = abs(r_max);
    }
    else if (r_min >= r_max)
    {
        ROS_ERROR("r_max must be greater than r_min!");
    }

    theta = ((double)rand() / RAND_MAX) * 2 * M_PI;
    psi = ((double)rand() / RAND_MAX) * 2 * M_PI;
    radius = ((double)rand() / RAND_MAX) * (r_max - r_min) + r_min;

    // view_origin.setX(radius * sin(theta) * cos(psi));
    // view_origin.setY(radius * sin(theta) * sin(psi));
    // view_origin.setZ(radius * cos(theta));

    view_origin.setX(radius * sin(theta) * cos(psi) + observation_center.getX());
    view_origin.setY(radius * sin(theta) * sin(psi) + observation_center.getY());
    view_origin.setZ(radius * cos(theta) + observation_center.getZ());

    z_direction.setX(observation_center.getX() - view_origin.getX());
    z_direction.setY(observation_center.getY() - view_origin.getY());
    z_direction.setZ(observation_center.getZ() - view_origin.getZ());

    z_direction.normalize();

    // random vector just to find one that's perpendicular to z_direction
    //   rand_vector.setX((double)rand() / RAND_MAX);
    //   rand_vector.setY((double)rand() / RAND_MAX);
    //   rand_vector.setZ((double)rand() / RAND_MAX);

    rand_vector.setX(1);
    rand_vector.setY(0);
    rand_vector.setZ(0);
    rand_vector.normalize();

    y_direction = z_direction.cross(rand_vector);
    y_direction.setZ(-1 * abs(y_direction.getZ()));
    y_direction.normalize();

    x_direction = y_direction.cross(z_direction);
    x_direction.normalize();

    //   rotation_matrix.setValue(x_direction.getX(), x_direction.getY(), x_direction.getZ(), y_direction.getX(),
    //                            y_direction.getY(), y_direction.getZ(), z_direction.getX(), z_direction.getY(),
    //                            z_direction.getZ());

    rotation_matrix.setValue(x_direction.getX(), y_direction.getX(), z_direction.getX(), x_direction.getY(),
                             y_direction.getY(), z_direction.getY(), x_direction.getZ(), y_direction.getZ(),
                             z_direction.getZ());

    rotation_matrix.getRotation(view_orientation);
    view_orientation.normalize();

    view_pose.setOrigin(view_origin);
    view_pose.setRotation(view_orientation);

#if 0

  geometry_msgs::Point x, y, z, randv, view_oriin;
  geometry_msgs::Quaternion quat;
  geometry_msgs::Vector3 x_line, y_line, z_line;
  tf::pointTFToMsg(rand_vector, randv);
  tf::pointTFToMsg(z_direction, z);
  tf::pointTFToMsg(y_direction, y);
  tf::pointTFToMsg(x_direction, x);
  tf::pointTFToMsg(view_origin, view_oriin);
  tf::quaternionTFToMsg(view_orientation, quat);
  tf::vector3TFToMsg(rotation_matrix.getRow(0), x_line);
  tf::vector3TFToMsg(rotation_matrix.getRow(1), y_line);
  tf::vector3TFToMsg(rotation_matrix.getRow(2), z_line);

  cout << "VIEW ORIGIN" << endl << view_oriin << endl;
  cout << "RAND" << endl << randv << endl;
  cout << "Z" << endl << z << endl;
  cout << "Y" << endl << y << endl;
  cout << "X" << endl << x << endl;
  cout << "QUAT" << endl << quat << endl;
  cout << "LINE X" << endl << x_line << endl;
  cout << "LINE Y" << endl << y_line << endl;
  cout << "LINE Z" << endl << z_line << endl;

  tf::Vector3 test_z;
  geometry_msgs::Point zz;

  test_z = x_direction.cross(y_direction);
  tf::pointTFToMsg(test_z, zz);
  cout << "IS THIS Z?" << endl << zz << endl;

#endif

    return view_pose;
}

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

            tf_pose = genPose(r_min, r_max, tf_point);

            tf::poseTFToMsg(tf_pose, one_pose);

            point_poses.poses.push_back(one_pose);

            // ROS_INFO("Pose published: %d", n);
        }
    }

    return point_poses;
}

void evalPose(geometry_msgs::Pose geom_pose)
{
    tf::Pose tf_pose;
    octomath::Pose6D octo_pose;

    tf::poseMsgToTF(geom_pose, tf_pose);
    octo_pose = octomap::poseTfToOctomap(tf_pose);

    octomath::Vector3 start_point, direction, end_point;

    start_point.x() = octo_pose.x();
    start_point.y() = octo_pose.y();
    start_point.z() = octo_pose.z();

    direction.x() = octo_pose.roll();
    direction.y() = octo_pose.pitch();
    direction.z() = octo_pose.yaw();

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publisher");

    ros::NodeHandle n;

    ros::Publisher pub_poseArray = n.advertise<geometry_msgs::PoseArray>("/pose_array", 10);

    // ros::Subscriber sub = n.subscribe("/octomap_full", 1, octomapCallback);

    // ros::Subscriber sub_spaceDefined = n.subscribe<visualization_msgs::MarkerArray>("/cells_vis_array", 10, mapCallback);

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
