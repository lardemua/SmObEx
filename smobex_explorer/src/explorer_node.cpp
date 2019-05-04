#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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

sensor_msgs::PointCloud2 cloud_clusters_publish;
sensor_msgs::PointCloud2 centroid_clusters_publish;
float octomap_resolution = 0.1;

using namespace std;

bool customRegionGrowing(const PointTypeIO &point_a, const PointTypeIO &point_b, float squared_distance)
{
	if (squared_distance < (octomap_resolution * 2) * (octomap_resolution * 2) * 1.1)
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
	std::vector<geometry_msgs::Point> centroids_vect;

	// Data containers used
	pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>);
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);

	pcl::PointXYZRGBA point_add_rgba;
	pcl::PointXYZ point_add, centroid_pcl;
	pcl::PointCloud<pcl::PointXYZ> all_centroids_pcl;

	// Load the input point cloud
	pcl::fromROSMsg(*unknown_cloud, *cloud_out);

	// Set up a Conditional Euclidean Clustering class
	pcl::ConditionalEuclideanClustering<PointTypeIO> cec(true);
	cec.setInputCloud(cloud_out);
	cec.setConditionFunction(&customRegionGrowing);
	cec.setClusterTolerance(octomap_resolution * 3);
	cec.segment(*clusters);

	for (int i = 0; i < clusters->size(); ++i)
	{
		pcl::CentroidPoint<pcl::PointXYZ> centroid_points;
		geometry_msgs::Point centroid;

		int label_r = ((double)rand() / RAND_MAX) * 255;
		int label_g = ((double)rand() / RAND_MAX) * 255;
		int label_b = ((double)rand() / RAND_MAX) * 255;

		for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
		{
			cloud_out->points[(*clusters)[i].indices[j]].r = label_r;
			cloud_out->points[(*clusters)[i].indices[j]].g = label_g;
			cloud_out->points[(*clusters)[i].indices[j]].b = label_b;

			point_add_rgba = cloud_out->points[(*clusters)[i].indices[j]];

			pcl::copyPoint(point_add_rgba, point_add);

			centroid_points.add(point_add);
		}

		centroid_points.get(centroid_pcl);

		centroid.x = centroid_pcl.x;
		centroid.y = centroid_pcl.y;
		centroid.z = centroid_pcl.z;

		centroids_vect.push_back(centroid);

		all_centroids_pcl.push_back(centroid_pcl);
	}

	// Save the output point cloud
	// ROS_INFO("SAVING PCL");
	// pcl::io::savePCDFile("output.pcd", *cloud_out);

	pcl::toROSMsg(*cloud_out, cloud_clusters_publish);
	pcl::toROSMsg(all_centroids_pcl, centroid_clusters_publish);

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
	// y_direction.setZ(-1 * abs(y_direction.getZ()));
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
	geometry_msgs::PoseStamped best_pose;

	visualization_msgs::Marker arrow;

	std::vector<geometry_msgs::Point> clusters_centroids;

	static const std::string PLANNING_GROUP = "manipulator";

	ros::init(argc, argv, "explorer_node");

	ros::AsyncSpinner spinner(1); // TODO see if improves performance
	spinner.start();

	ros::NodeHandle n;

	ros::param::get("/octomap_server_node/resolution", octomap_resolution);

	ros::Publisher pub_cloud_clusters = n.advertise<sensor_msgs::PointCloud2>("/clusters_cloud", 10);
	ros::Publisher pub_centers_clusters = n.advertise<sensor_msgs::PointCloud2>("/clusters_centers", 10);

	ros::Publisher pub_arrows = n.advertise<visualization_msgs::MarkerArray>("/pose_arrows", 10);
	ros::Publisher pub_space = n.advertise<visualization_msgs::MarkerArray>("/discovered_space", 10);

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	const robot_state::JointModelGroup *joint_model_group =
			move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std_msgs::ColorRGBA green_color;
	green_color.r = 0.0;
	green_color.g = 1.0;
	green_color.b = 0.0;
	green_color.a = 1.0;

	// int step = 1;
	float min_range = 0;
	float max_range = 1;
	float width_FOV = M_PI;
	float height_FOV = M_PI;
	std::string frame_id = "/world";

	// ros::param::get("~" + ros::names::remap("step"), step);
	ros::param::get("~" + ros::names::remap("min_range"), min_range);
	ros::param::get("~" + ros::names::remap("max_range"), max_range);
	ros::param::get("~" + ros::names::remap("width_FOV"), width_FOV);
	ros::param::get("~" + ros::names::remap("height_FOV"), height_FOV);
	ros::param::get("~" + ros::names::remap("frame_id"), frame_id);

	// evaluatePose pose_test(20, 0.8, 3.5, 58 * M_PI / 180, 45 * M_PI / 180);
	// evaluatePose pose_test(step, min_range, max_range, width_FOV, height_FOV);
	evaluatePose pose_test(min_range, max_range, width_FOV, height_FOV);

	int n_poses = 20;
	float threshold = 0.01;
	float max_reach = 0.951;

	ros::param::get("~n_poses", n_poses);
	ros::param::get("~threshold", threshold);

	int arrow_id = -1;
	float best_score = 1;
	int best_arrow_id;

	srand(time(NULL));

	//do
	while (best_score > threshold)
	{
		visualization_msgs::MarkerArray all_poses;
		visualization_msgs::MarkerArray single_view_boxes;

		arrow_id = -1;
		best_score = -1;

		move_group.clearPoseTargets();

		sensor_msgs::PointCloud2ConstPtr unknown_cloud =
				ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/unknown_pc", n);

		pose_test.writeKnownOctomap();
		pose_test.writeUnknownOctomap();

		pose_test.writeUnknownCloud();

		clusters_centroids = findClusters(unknown_cloud);

		if (clusters_centroids.size() > 0)
		{
			pub_cloud_clusters.publish(cloud_clusters_publish);

			centroid_clusters_publish.header.stamp = ros::Time::now();
			centroid_clusters_publish.header.frame_id = frame_id;

			pub_centers_clusters.publish(centroid_clusters_publish);
		}

		move_group.setPlanningTime(0.4);
		const std::string end_effector_link = move_group.getEndEffectorLink();

		size_t total_clusters = clusters_centroids.size();

		size_t poses_by_cluster = n_poses / total_clusters;

		ROS_INFO_STREAM("Number of clusters: " << total_clusters);
		ROS_INFO_STREAM("Poses by cluster: " << poses_by_cluster);

		for (size_t cluster_idx = 0; cluster_idx < total_clusters; cluster_idx++)
		{
			geometry_msgs::Point observation_point = clusters_centroids[cluster_idx];

			// size_t pose_idx = 0;

			// while (pose_idx < poses_by_cluster)
			// #pragma omp parallel for //TODO
			for (size_t pose_idx = 0; pose_idx < poses_by_cluster; pose_idx++)
			{
				geometry_msgs::PoseStamped target_pose = move_group.getRandomPose();
				target_pose.pose.position.x = abs(target_pose.pose.position.x);

				// double pose_dist;

				// geometry_msgs::PoseStamped target_pose ;

				// do
				// {
				// 	target_pose = move_group.getRandomPose();
				// 	target_pose.pose.position.x = abs(target_pose.pose.position.x);

				// 	float x = target_pose.pose.position.x;
				// 	float y = target_pose.pose.position.y;
				// 	float z = target_pose.pose.position.z;

				// 	pose_dist = sqrt(x * x + y * y + z * z);

				// } while (pose_dist > max_range * 0.8);

				// if (target_pose.pose.position.x < 0.2)
				// {
				// 	target_pose.pose.position.x = 0.2;
				// }

				geometry_msgs::Quaternion quat_orient = getOrientation(target_pose, observation_point);
				target_pose.pose.orientation = quat_orient;

				bool set_target = move_group.setJointValueTarget(target_pose, end_effector_link);

				bool set_plan = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

				ROS_INFO("Plan (pose goal) %s", set_plan ? "SUCCESS" : "FAILED");
				ROS_INFO("Target (pose goal) %s", set_target ? "SUCCESS" : "FAILED");

				ROS_INFO_STREAM("Cluster " << cluster_idx + 1 << " of " << total_clusters << " Pose " << pose_idx + 1 << " of " << poses_by_cluster);

				tf::poseMsgToTF(target_pose.pose, pose_test.view_pose);

				arrow.header.stamp = ros::Time::now();
				arrow.header.frame_id = frame_id;

				arrow.id = ++arrow_id;

				arrow.type = visualization_msgs::Marker::ARROW;

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

				if (set_target && set_plan)
				{
					pose_test.evalPose();

					ROS_INFO_STREAM("Score: " << pose_test.score);

					arrow.color = pose_test.score_color;

					if (pose_test.score > best_score)
					{
						best_score = pose_test.score;
						best_pose = target_pose;
						best_arrow_id = arrow_id;

						single_view_boxes = pose_test.discoveredBoxesVis(frame_id);
					}

					// pose_idx++;
				}
				else
				{
					pose_test.score = 0;

					ROS_INFO_STREAM("Score: " << pose_test.score);

					arrow.color = pose_test.score_color;
				}

				all_poses.markers.push_back(arrow);
				pub_arrows.publish(all_poses);

				ROS_INFO("---------");
			}
		}

		all_poses.markers[best_arrow_id].color = green_color;
		all_poses.markers[best_arrow_id].scale.x *= 2;
		all_poses.markers[best_arrow_id].scale.y *= 2;
		all_poses.markers[best_arrow_id].scale.z *= 2;

		pub_arrows.publish(all_poses);
		pub_space.publish(single_view_boxes);

		ROS_WARN("MOVING!!!");

		// move_group.setPlanningTime(1);
		// move_group.setNumPlanningAttempts(10);

		move_group.setJointValueTarget(best_pose, end_effector_link);
		// move_group.plan(my_plan);

		bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO("Plan (best pose goal) %s", success ? "SUCCESS" : "FAILED");

		ROS_INFO("---------");

		for (size_t id_arrow_mrk = 0; id_arrow_mrk < all_poses.markers.size(); id_arrow_mrk++)
		{
			all_poses.markers[id_arrow_mrk].action = visualization_msgs::Marker::DELETEALL;
		}
		pub_arrows.publish(all_poses);

		for (size_t id_vol = 0; id_vol < single_view_boxes.markers.size(); id_vol++)
		{
			single_view_boxes.markers[id_vol].action = visualization_msgs::Marker::DELETEALL;
		}
		pub_space.publish(single_view_boxes);

	} //while (best_score > threshold);

	ROS_INFO_STREAM("Final best score: " << best_score);

	ros::shutdown();

	return 0;
}
