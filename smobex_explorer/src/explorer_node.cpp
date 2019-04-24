#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <smobex_explorer/explorer.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

typedef pcl::PointXYZRGBA PointTypeIO;

using namespace std;

bool customRegionGrowing(const PointTypeIO &point_a, const PointTypeIO &point_b, float squared_distance)
{
  if (squared_distance < 0.1 * 0.1 * 1.1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<geometry_msgs::Point> findClusters(sensor_msgs::PointCloud2ConstPtr unknown_cloud)
{
  ROS_INFO("FINDING CLUSTERS");

  std::vector<geometry_msgs::Point> centroids_vect;

  // Data containers used
  pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>);
  pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);

  pcl::PointXYZRGBA point_add_rgba;
  pcl::PointXYZ point_add, centroid_pcl;

  // Load the input point cloud
  pcl::fromROSMsg(*unknown_cloud, *cloud_out);

  // Set up a Conditional Euclidean Clustering class
  pcl::ConditionalEuclideanClustering<PointTypeIO> cec(true);
  cec.setInputCloud(cloud_out);
  cec.setConditionFunction(&customRegionGrowing);
  cec.setClusterTolerance(0.20);  // TODO
  cec.segment(*clusters);

  ROS_INFO_STREAM("N clusters " << clusters->size());

  for (int i = 0; i < clusters->size(); ++i)
  {
    pcl::CentroidPoint<pcl::PointXYZ> centroid_points;
    geometry_msgs::Point centroid;

    // int label_r = ((double)rand() / RAND_MAX) * 255;
    // int label_g = ((double)rand() / RAND_MAX) * 255;
    // int label_b = ((double)rand() / RAND_MAX) * 255;

    for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
    {
      // cloud_out->points[(*clusters)[i].indices[j]].r = label_r;
      // cloud_out->points[(*clusters)[i].indices[j]].g = label_g;
      // cloud_out->points[(*clusters)[i].indices[j]].b = label_b;

      point_add_rgba = cloud_out->points[(*clusters)[i].indices[j]];

      pcl::copyPoint(point_add_rgba, point_add);

      centroid_points.add(point_add);
    }

    centroid_points.get(centroid_pcl);

    centroid.x = centroid_pcl.x;
    centroid.y = centroid_pcl.y;
    centroid.z = centroid_pcl.z;

    centroids_vect.push_back(centroid);
  }

  // Save the output point cloud
  // ROS_INFO("SAVING PCL");
  // pcl::io::savePCDFile("output.pcd", *cloud_out);

  return centroids_vect;
}

geometry_msgs::Quaternion getOrientation(geometry_msgs::PoseStamped pose, geometry_msgs::Point point)
{
  tf::Vector3 z_direction, y_direction, x_direction, rand_vector;
  tf::Matrix3x3 rotation_matrix;
  tf::Quaternion view_orientation;

  geometry_msgs::Quaternion quat_out;

#if 0

  pcl::PointCloud<pcl::PointXYZ> unknown_pcl;

  pcl::fromROSMsg(*unknown_cloud, unknown_pcl);

  size_t n_points = unknown_pcl.size();
  pcl::PointXYZ point_pcl;

    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = unknown_pcl.begin(); it != unknown_pcl.end(); it++)
    {
        centroid.add(*it);
    }

    centroid.get(point_pcl);

#elif 0

  int pt_number = ((double)rand() / RAND_MAX) * n_points;
  point_pcl = unknown_pcl.at(pt_number);

  float xc = point_pcl.x;
  float yc = point_pcl.y;
  float zc = point_pcl.z;

#endif

  float xc = point.x;
  float yc = point.y;
  float zc = point.z;

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

  sensor_msgs::PointCloud2ConstPtr unknown_cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/unknown_pc", n);

  ros::Publisher pub_arrows = n.advertise<visualization_msgs::MarkerArray>("/pose_arrows", 10);

  srand(time(NULL));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::vector<geometry_msgs::Point> clusters_centroids = findClusters(unknown_cloud);

  evaluatePose pose_test(20, 0.8, 10, 58 * M_PI / 180, 45 * M_PI / 180);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::PoseStamped target_pose, best_pose;
  visualization_msgs::MarkerArray all_poses;
  geometry_msgs::Point observation_point;
  visualization_msgs::Marker arrow;

  pose_test.writeKnownOctomap();
  pose_test.writeUnknownOctomap();

  move_group.setPlanningTime(0.3);

  float best_score = -1;
  int best_pose_n = -1;
  int best_cluster_n = -1;
  int n_poses = 20;
  int arrow_id = -1;

  for (size_t i = 0; i < clusters_centroids.size(); i++)
  {
    observation_point = clusters_centroids[i];

    for (size_t n = 0; n < n_poses; n++)
    {
      target_pose = move_group.getRandomPose();

      geometry_msgs::Quaternion quat_orient = getOrientation(target_pose, observation_point);
      target_pose.pose.orientation = quat_orient;

      move_group.setPoseTarget(target_pose);

      ros::Time t = ros::Time::now();

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ros::Duration d = (ros::Time::now() - t);
      ROS_INFO_STREAM("Planning took " << d.toSec() << " secs.");

      ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      if (success)
      {
        tf::poseMsgToTF(target_pose.pose, pose_test.view_pose);

        t = ros::Time::now();
        pose_test.evalPose();
        d = (ros::Time::now() - t);
        ROS_INFO_STREAM("Evaluation took " << d.toSec() << " secs.");

        ROS_INFO_STREAM("Pose " << i << " score: " << pose_test.score);

        if (pose_test.score > best_score)
        {
          best_cluster_n = i;
          best_pose_n = n;
          best_score = pose_test.score;
          best_pose = target_pose;
          ROS_INFO_STREAM("Best pose now: Cluster " << i << " Pose " << n);
          ROS_INFO_STREAM("Target pose now: " << target_pose);
        }

        arrow.header.stamp = ros::Time::now();
        arrow.header.frame_id = "/base_link";

        arrow.id = ++arrow_id;

        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.color = pose_test.score_color;

        arrow.pose.position = target_pose.pose.position;

        tf::Quaternion q_rot, q_new;
        geometry_msgs::Quaternion q_arrow;
        tf::quaternionMsgToTF(target_pose.pose.orientation, q_new);

        q_rot.setRPY(0, -M_PI / 2, 0);
        q_new = q_new * q_rot;
        q_new.normalize();

        tf::quaternionTFToMsg(q_new, q_arrow);
        arrow.pose.orientation = q_arrow;

        arrow.scale.x = 0.10;
        arrow.scale.y = 0.02;
        arrow.scale.z = 0.02;

        all_poses.markers.push_back(arrow);

        pub_arrows.publish(all_poses);
      }

      ROS_INFO("---------");
    }
  }

  /*
    for (size_t i = 0; i < 20; i++)
    {
      target_pose = move_group.getRandomPose();

      geometry_msgs::Quaternion quat_orient = getOrientation(target_pose);
      target_pose.pose.orientation = quat_orient;

      move_group.setPoseTarget(target_pose);

      ros::Time t = ros::Time::now();

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ros::Duration d = (ros::Time::now() - t);
      ROS_INFO_STREAM("Planning took " << d.toSec() << " secs.");

      ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      if (success)
      {
        tf::poseMsgToTF(target_pose.pose, pose_test.view_pose);

        t = ros::Time::now();
        pose_test.evalPose();
        d = (ros::Time::now() - t);
        ROS_INFO_STREAM("Evaluation took " << d.toSec() << " secs.");

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

        tf::Quaternion q_rot, q_new;
        geometry_msgs::Quaternion q_arrow;
        tf::quaternionMsgToTF(target_pose.pose.orientation, q_new);

        q_rot.setRPY(0, -M_PI / 2, 0);
        q_new = q_new * q_rot;
        q_new.normalize();

        tf::quaternionTFToMsg(q_new, q_arrow);
        arrow.pose.orientation = q_arrow;

        arrow.scale.x = 0.10;
        arrow.scale.y = 0.02;
        arrow.scale.z = 0.02;

        all_poses.markers.push_back(arrow);

        pub_arrows.publish(all_poses);
      }

      ROS_INFO("---------");
    } */

  ROS_INFO_STREAM("Best score was " << best_score << " of pose " << best_pose_n << " in cluster " << best_cluster_n);
  ROS_INFO_STREAM("Best pose: " << best_pose);
  move_group.clearPoseTargets();
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

  ros::shutdown();

  return 0;
}
