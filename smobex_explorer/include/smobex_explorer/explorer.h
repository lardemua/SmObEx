#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#include <ros/ros.h>
#include <cmath>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl-1.8/pcl/filters/frustum_culling.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

// #include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include <colormap/colormap.h>

// #include "Eigen/Core"
// #include "Eigen/Geometry"

class generatePose
{
public:
	tf::Pose view_pose;

	// tf::Pose genPose(float r_min, float r_max, tf::Point observation_center)
	void genPose(float r_min, float r_max, tf::Point observation_center)
	{
		// tf::Pose view_pose;
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
		rand_vector.setX((double)rand() / RAND_MAX);
		rand_vector.setY((double)rand() / RAND_MAX);
		rand_vector.setZ((double)rand() / RAND_MAX);

		// rand_vector.setX(1);
		// rand_vector.setY(0);
		// rand_vector.setZ(0);
		// rand_vector.normalize();

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

		// return view_pose;
	}
};

class unknownVoxel
{

public:
	octomap::OcTreeKey key;
	octomap::point3d center;
	double distance_to_camera;
	bool to_visit;
	size_t map_key;

	unknownVoxel()
	{
		to_visit = true;
	}

	bool operator==(const unknownVoxel &rhs) const { return this->key == rhs.key; }
	bool operator==(const octomap::OcTreeKey &rhs) const { return this->key == rhs; }
};

typedef std::map<size_t, unknownVoxel> map_type;

bool compareVoxelDistance(unknownVoxel const &a, unknownVoxel const &b)
{
	return a.distance_to_camera > b.distance_to_camera;
	// return a->second.distance_to_camera > b->second.distance_to_camera;
}

bool compareVoxelDistanceIterator(map_type::iterator const &a, map_type::iterator const &b)
{
	// return a.distance_to_camera > b.distance_to_camera;
	return a->second.distance_to_camera > b->second.distance_to_camera;
}

class evaluatePose : public generatePose
{
public:
	float score = 0;
	std_msgs::ColorRGBA score_color;

	int step;
	float min_range;
	float max_range;
	float width_FOV;
	float height_FOV;
	int pix_width;
	int pix_height;

	octomap::OcTree *octree = NULL;
	octomap::OcTree *unknown_octree = NULL;
	octomap::point3d_list ray_points_list;
	octomap::KeySet first_keys;
	octomap::KeySet posterior_keys;

	pcl::PointCloud<pcl::PointXYZ> rays_point_cloud_world;
	pcl::PointCloud<pcl::PointXYZ> unknown_centers_pcl;

	octomap::point3d min_bbx, max_bbx;

	evaluatePose(/*int _step,*/ float _min_range, float _max_range, float _width_FOV, float _height_FOV)
	{
		min_range = _min_range;
		max_range = _max_range;
		width_FOV = _width_FOV;
		height_FOV = _height_FOV;

		ros::param::get("x_max", max_bbx.x());
		ros::param::get("y_max", max_bbx.y());
		ros::param::get("z_max", max_bbx.z());

		ros::param::get("x_min", min_bbx.x());
		ros::param::get("y_min", min_bbx.y());
		ros::param::get("z_min", min_bbx.z());
	}

	evaluatePose(int _step, float _min_range, float _max_range, float _width_FOV, float _height_FOV)
	{
		step = _step;
		min_range = _min_range;
		max_range = _max_range;
		width_FOV = _width_FOV;
		height_FOV = _height_FOV;

		ros::NodeHandle n;
		sensor_msgs::CameraInfoConstPtr CamInfo;

		CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info", n,
																	  ros::Duration(10));

		pix_width = CamInfo->width;
		pix_height = CamInfo->height;

		float delta_rad_w = width_FOV / pix_width;
		float delta_rad_h = height_FOV / pix_height;

		float corner_rad_w = width_FOV / 2;
		float corner_rad_h = height_FOV / 2;

		// generate the points of the camera rays
		float rad_h = (M_PI - height_FOV) / 2;

		for (int row_pix = 0; row_pix < pix_height; row_pix += step)
		{
			float rad_w = (M_PI - width_FOV) / 2;

			for (int col_pix = 0; col_pix < pix_width; col_pix += step)
			{
				float x = 1 * sin(rad_h) * cos(rad_w);
				float z = 1 * sin(rad_h) * sin(rad_w);
				float y = 1 * cos(rad_h);

				rays_point_cloud_world.push_back(pcl::PointXYZ(x, y, z));

				rad_w += delta_rad_w * step;
			}
			rad_h += delta_rad_h * step;
		}

		// rays_point_cloud.push_back(pcl::PointXYZ(-0.01, 0, 0.8));
		// rays_point_cloud.push_back(pcl::PointXYZ(0.01, 0, 0.8));
		// pcl_ros::transformPointCloud(rays_point_cloud, rays_point_cloud, view_pose);

		ros::param::get("x_max", max_bbx.x());
		ros::param::get("y_max", max_bbx.y());
		ros::param::get("z_max", max_bbx.z());

		ros::param::get("x_min", min_bbx.x());
		ros::param::get("y_min", min_bbx.y());
		ros::param::get("z_min", min_bbx.z());
	}

	// void writeKnownOctomapCallback(const octomap_msgs::OctomapConstPtr &map)
	// {
	//     using namespace octomap;

	//     // ROS_INFO("I'm inside the callback");

	//     AbstractOcTree *tree = NULL;

	//     if (octree != NULL)
	//     {
	//         // ROS_INFO("DEBUG: Going to DEL OCTREE");
	//         delete (octree);
	//         // ROS_INFO("DEBUG: Octree DEL");
	//     }

	//     tree = msgToMap(*map);
	//     octree = dynamic_cast<OcTree *>(tree);
	// }

	// void writeUnknownOctomapCallback(const octomap_msgs::OctomapConstPtr &map)
	// {
	//     using namespace octomap;

	//     // ROS_INFO("I'm inside the callback");

	//     AbstractOcTree *tree = NULL;

	//     if (unknown_octree != NULL)
	//     {
	//         // ROS_INFO("DEBUG: Going to DEL OCTREE");
	//         delete (unknown_octree);
	//         // ROS_INFO("DEBUG: Octree DEL");
	//     }

	//     tree = msgToMap(*map);
	//     unknown_octree = dynamic_cast<OcTree *>(tree);
	// }

	void writeKnownOctomap()
	{
		using namespace octomap;

		ros::NodeHandle n;

		AbstractOcTree *tree = NULL;

		if (octree != NULL)
		{
			delete (octree);
		}

		octomap_msgs::OctomapConstPtr map = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_full", n);
		tree = msgToMap(*map);
		octree = dynamic_cast<OcTree *>(tree);
	}

	void writeUnknownOctomap()
	{
		using namespace octomap;

		ros::NodeHandle n;

		AbstractOcTree *tree = NULL;

		if (unknown_octree != NULL)
		{
			delete (unknown_octree);
		}

		octomap_msgs::OctomapConstPtr map = ros::topic::waitForMessage<octomap_msgs::Octomap>("/unknown_full_map", n);
		tree = msgToMap(*map);
		unknown_octree = dynamic_cast<OcTree *>(tree);
	}

	void writeUnknownCloud()
	{
		ros::NodeHandle n;

		sensor_msgs::PointCloud2ConstPtr unknown_cloud =
			ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/unknown_pc", n);

		pcl::fromROSMsg(*unknown_cloud, unknown_centers_pcl);
	}

	void evalPose()
	{
		using namespace octomap;
		using namespace octomath;

		Vector3 origin;

		while (octree == NULL || unknown_octree == NULL)
		{
			ROS_WARN("No OcTrees... Did you call the writting functions? Calling them automatically.");

			writeKnownOctomap();
			writeUnknownOctomap();
		}

		first_keys.clear();
		posterior_keys.clear();

		Pose6D octo_pose = poseTfToOctomap(view_pose);

		origin.x() = octo_pose.x();
		origin.y() = octo_pose.y();
		origin.z() = octo_pose.z();

		if (octree != NULL && unknown_octree != NULL)
		{

			pcl::FrustumCulling<pcl::PointXYZ> fc;
			fc.setInputCloud(unknown_centers_pcl.makeShared());
			fc.setVerticalFOV(height_FOV * 180 / M_PI);
			fc.setHorizontalFOV(width_FOV * 180 / M_PI);
			fc.setNearPlaneDistance(min_range);
			fc.setFarPlaneDistance(max_range);

			Eigen::Affine3d pose_origin_affine;
			Eigen::Matrix4d pose_orig;
			Eigen::Matrix4d cam2robot;

			tf::poseTFToEigen(view_pose, pose_origin_affine);

			pose_orig = pose_origin_affine.matrix();

			cam2robot << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

			Eigen::Matrix4d pose_new_d = pose_orig * cam2robot;
			Eigen::Matrix4f pose_new = pose_new_d.cast<float>();

			fc.setCameraPose(pose_new);

			pcl::PointCloud<pcl::PointXYZ> points_inside;
			fc.filter(points_inside);

			std::vector<map_type::iterator> unknown_voxels;
			map_type map_unknown_voxels;

			// ROS_INFO_STREAM("N points unknown: " << points_inside.size());

			for (pcl::PointCloud<pcl::PointXYZ>::iterator it = points_inside.begin(); it != points_inside.end(); it++)
			{
				unknownVoxel a_voxel;
				octomap::OcTreeKey::KeyHash hash;

				a_voxel.key = unknown_octree->coordToKey(it->x, it->y, it->z);
				a_voxel.center = Vector3(it->x, it->y, it->z);
				a_voxel.distance_to_camera = origin.distance(Vector3(it->x, it->y, it->z));
				a_voxel.map_key = hash(a_voxel.key);

				map_unknown_voxels.insert(std::pair<size_t, unknownVoxel>(a_voxel.map_key, a_voxel));

				map_type::iterator map_it = map_unknown_voxels.find(a_voxel.map_key);

				unknown_voxels.push_back(map_it);
			}

			std::sort(unknown_voxels.begin(), unknown_voxels.end(), compareVoxelDistanceIterator);

			KeyRay ray_keys_before, ray_keys_after;
			Vector3 voxel_center, end_point, direction;

			// ros::Time t = ros::Time::now();
			// ros::Time t_for;
			// ros::Duration d_for_total(0);

			// t_for = ros::Time::now();
			// double total1 = 0;
			// double total2 = 0;
			// double total3 = 0;
			// double total4 = 0;

			for (size_t idx = 0; idx < unknown_voxels.size(); idx++)
			{
				octomap::OcTreeKey::KeyHash hash;

				if (map_unknown_voxels[unknown_voxels[idx]->second.map_key].to_visit == false)
				{
					continue;
				}

				voxel_center = unknown_octree->keyToCoord(unknown_voxels[idx]->second.key);

				direction = voxel_center - origin;
				bool occupied = octree->castRay(origin, direction, end_point, true, max_range);

				unknown_octree->computeRayKeys(origin, end_point, ray_keys_before);

				bool first = true;

				// ros::Time tic, tic_inside;
				for (KeyRay::iterator it_key = ray_keys_before.begin(); it_key != ray_keys_before.end(); it_key++)
				{

					// tic = ros::Time::now();
					bool C1 = origin.distance(unknown_octree->keyToCoord(*it_key)) >= min_range;
					// total1 += (ros::Time::now() - tic).toSec();

					if (C1)
					{

						// tic = ros::Time::now();
						bool C2 = unknown_octree->search(*it_key);
						// total2 += (ros::Time::now() - tic).toSec();

						// tic = ros::Time::now();
						if (C2)
						{

							if (first)
							{
								first_keys.insert(*it_key);
								first = false;
							}
							else
							{
								posterior_keys.insert(*it_key);
							}

							ray_points_list.push_back(origin);
							ray_points_list.push_back(end_point);

							// tic_inside = ros::Time::now();

							map_type::iterator map_it = map_unknown_voxels.find(hash(*it_key));

							if (map_it != map_unknown_voxels.end())
							{
								map_it->second.to_visit = false;
							}

							// total4 += (ros::Time::now() - tic_inside).toSec();
						}
					}
					// total3 += (ros::Time::now() - tic).toSec();
				}
				// d_for_total += (ros::Time::now() - t_for);

				if (occupied)
				{
					unknown_octree->computeRayKeys(end_point, voxel_center, ray_keys_after);

					for (KeyRay::iterator it_key = ray_keys_after.begin(); it_key != ray_keys_after.end(); it_key++)
					{
						map_type::iterator map_it = map_unknown_voxels.find(hash(*it_key));

						if (map_it != map_unknown_voxels.end())
						{
							map_it->second.to_visit = false;
						}
					}
				}
			}
			// ros::Duration d = ros::Time::now() - t;
			// ROS_INFO_STREAM("for iterator: " << d_for_total.toSec());
			// ROS_INFO_STREAM("1: " << total1);
			// ROS_INFO_STREAM("2: " << total2);
			// ROS_INFO_STREAM("3: " << total3);
			// ROS_INFO_STREAM("4: " << total4);
			// // ROS_INFO_STREAM("New for: " << d.toSec() << " secs. N points final: " << unknown_voxels.size());

			// ROS_INFO_STREAM("Checked voxels: " << checked_voxels);

			getScore();
		}
		else
		{
			ROS_INFO("No OcTree.");
		}
	}

	void evalPosePixelBased()
	{
		using namespace octomap;
		using namespace octomath;

		Vector3 origin;

		while (octree == NULL || unknown_octree == NULL)
		{
			ROS_WARN("No OcTrees... Did you call the writting functions? Calling them automatically.");

			writeKnownOctomap();
			writeUnknownOctomap();
		}

		first_keys.clear();
		posterior_keys.clear();

		Pose6D octo_pose = poseTfToOctomap(view_pose);

		origin.x() = octo_pose.x();
		origin.y() = octo_pose.y();
		origin.z() = octo_pose.z();

		pcl::PointCloud<pcl::PointXYZ> rays_point_cloud;
		pcl_ros::transformPointCloud(rays_point_cloud_world, rays_point_cloud, view_pose);

		int n_start_points = rays_point_cloud.height * rays_point_cloud.width;
		if (octree != NULL && unknown_octree != NULL)
		{

			ros::Duration d, d1;
			ros::Time t;
			ros::Time t1 = ros::Time::now();
#pragma omp parallel for
			for (size_t i = 0; i < n_start_points; i++)
			{
				KeyRay ray_keys;
				pcl::PointXYZ point = rays_point_cloud.at(i);
				Vector3 start_point, direction, end_point;

				start_point.x() = point.x;
				start_point.y() = point.y;
				start_point.z() = point.z;

				direction = start_point - origin;

				t = ros::Time::now();
				octree->castRay(origin, direction, end_point, true, max_range);
				d = ros::Time::now() - t;
				ROS_INFO_STREAM("castRay: " << d.toSec() << "secs.");

				start_point = origin + direction.normalized() * min_range;

				t = ros::Time::now();
				octree->getRayIntersection(origin, direction, end_point, end_point);
				d = ros::Time::now() - t;
				ROS_INFO_STREAM("getRayIntersection: " << d.toSec() << "secs.");

				if (origin.distance(start_point) < origin.distance(end_point))
				{
					t = ros::Time::now();
					unknown_octree->computeRayKeys(start_point, end_point, ray_keys);
					d = ros::Time::now() - t;
					ROS_INFO_STREAM("computeRayKeys: " << d.toSec() << "secs.");

					bool first = true;
					t = ros::Time::now();
					for (KeyRay::iterator it = ray_keys.begin(); it != ray_keys.end(); it++)
					{
						if (unknown_octree->search(*it))
						{
							if (first)
							{
								first_keys.insert(*it);
								first = false;
							}
							else
							{
								posterior_keys.insert(*it);
							}
						}
					}
					d = ros::Time::now() - t;
					ROS_INFO_STREAM("iterator: " << d.toSec() << "secs.");

					t = ros::Time::now();
					ray_points_list.push_back(start_point);
					ray_points_list.push_back(end_point);
					d = ros::Time::now() - t;
					ROS_INFO_STREAM("pushBack: " << d.toSec() << "secs.");
				}
			}
			d1 = ros::Time::now() - t1;
			ROS_INFO_STREAM("#pragma took: " << d1.toSec() << "secs for " << n_start_points << " points.");

			for (KeySet::iterator it = posterior_keys.begin(); it != posterior_keys.end(); it++)
			{
				if (first_keys.find(*it) != first_keys.end())
				{
					posterior_keys.erase(*it);
				}
			}

			getScore();
		}

		else
		{
			ROS_INFO("No OcTree.");
		}
	}

	void getScore()
	{
		using namespace octomap;

		float resolution = octree->getResolution();
		float one_volume = resolution * resolution * resolution;

		float weight = 0.5;

		float found_volume = (first_keys.size() + posterior_keys.size() * weight) * one_volume;

		// third formula

		point3d deltas = max_bbx - min_bbx;

		float total_volume = deltas.x() * deltas.y() * deltas.z();
		float inner_volume =
			(deltas.x() - resolution * 2) * (deltas.y() - resolution * 2) * (deltas.z() - resolution * 2);
		float outer_volume = total_volume - inner_volume;

		float score_volume = outer_volume + inner_volume * weight;

		score = found_volume / score_volume;

		// class_colormap frustum_color("jet", 64, 1, true);
		getColor();
	}

	void getColor()
	{
		class_colormap frustum_color("jet", 64, 1, true);

		score_color = frustum_color.color(score * 64);
	}

	octomap::point3d_collection getDiscoveredCenters()
	{
		using namespace octomap;

		point3d_collection discovered_centers;

		for (KeySet::iterator it = posterior_keys.begin(); it != posterior_keys.end(); it++)
		{
			discovered_centers.push_back(unknown_octree->keyToCoord(*it));
		}

		return discovered_centers;
	}

	visualization_msgs::MarkerArray discoveredBoxesVis(std::string frame_id)
	{
		using namespace std;
		using namespace octomap;

		visualization_msgs::MarkerArray all_boxes;
		double unknown_octree_depth = unknown_octree->getTreeDepth();
		all_boxes.markers.resize(unknown_octree_depth + 1);

		ros::Time t = ros::Time::now();

		std_msgs::ColorRGBA blue;
		blue.r = 0.0;
		blue.g = 0.0;
		blue.b = 1.0;
		blue.a = 1.0;

		// std_msgs::ColorRGBA light_blue;;
		// light_blue.r = 0.0;
		// light_blue.g = 0.7;
		// light_blue.b = 1.0;
		// light_blue.a = 1.0;

		for (OcTree::iterator it = unknown_octree->begin(), end = unknown_octree->end(); it != end; ++it)
		{
			OcTreeKey node_key = it.getKey();

			if (first_keys.find(node_key) != first_keys.end())
			{
				double size = it.getSize();
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();

				unsigned idx = it.getDepth();
				assert(idx < all_boxes.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				all_boxes.markers[idx].points.push_back(cubeCenter);
				// all_boxes.markers[idx].color = blue;
				// all_boxes.markers[idx].colors.push_back(blue);
				// all_boxes.markers[idx].ns = "first_boxes";
			}
			else if (posterior_keys.find(node_key) != posterior_keys.end())
			{
				double size = it.getSize();
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();

				unsigned idx = it.getDepth();
				assert(idx < all_boxes.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				all_boxes.markers[idx].points.push_back(cubeCenter);
				// all_boxes.markers[idx].color = light_blue;
				// all_boxes.markers[idx].colors.push_back(light_blue);
				// all_boxes.markers[idx].ns = "posterior_boxes";
			}
		}

		for (unsigned i = 0; i < all_boxes.markers.size(); i++)
		{
			double size = unknown_octree->getNodeSize(i);

			all_boxes.markers[i].header.frame_id = frame_id;
			all_boxes.markers[i].header.stamp = t;
			all_boxes.markers[i].ns = "Pose";
			all_boxes.markers[i].id = i;
			all_boxes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			all_boxes.markers[i].scale.x = size;
			all_boxes.markers[i].scale.y = size;
			all_boxes.markers[i].scale.z = size;
			all_boxes.markers[i].color = blue;

			if (all_boxes.markers[i].points.size() > 0)
			{
				all_boxes.markers[i].action = visualization_msgs::Marker::ADD;
			}
			else
			{
				all_boxes.markers[i].action = visualization_msgs::Marker::DELETE;
			}
		}

		return all_boxes;
	}

	visualization_msgs::Marker rayLinesVis(std::string frame_id)
	{
		using namespace std;

		// if (ray_points_list.size() <= 0)
		// {
		//     this->evalPose();
		// }

		visualization_msgs::Marker line_vis;

		line_vis.header.frame_id = frame_id;
		line_vis.header.stamp = ros::Time::now();
		line_vis.ns = "Pose rays";
		line_vis.action = visualization_msgs::Marker::ADD;
		line_vis.pose.orientation.w = 1.0;
		line_vis.id = 0;
		line_vis.type = visualization_msgs::Marker::LINE_LIST;
		line_vis.scale.x = 0.001;

		line_vis.color.r = 0.5;
		line_vis.color.g = 0.5;
		line_vis.color.b = 0.5;
		line_vis.color.a = 1.0;

		for (octomap::point3d_list::iterator it = ray_points_list.begin(); it != ray_points_list.end(); it++)
		{
			geometry_msgs::Point point;
			point.x = it->octomath::Vector3::x();
			point.y = it->octomath::Vector3::y();
			point.z = it->octomath::Vector3::z();

			line_vis.points.push_back(point);
		}

		return line_vis;
	}

	visualization_msgs::Marker frustumLinesVis(std::string frame_id)
	{
		using namespace std;

		// if (ray_points_list.size() <= 0)
		// {
		//     this->evalPose();
		// }

		visualization_msgs::Marker line_vis;

		line_vis.header.frame_id = frame_id;
		line_vis.header.stamp = ros::Time::now();
		line_vis.ns = "Frustum";
		line_vis.action = visualization_msgs::Marker::ADD;
		line_vis.pose.orientation.w = 1.0;
		line_vis.id = 0;
		line_vis.type = visualization_msgs::Marker::LINE_LIST;
		line_vis.scale.x = 0.01;

		// line_vis.color.r = 0.2;
		// line_vis.color.g = 0.2;
		// line_vis.color.b = 0.2;
		// line_vis.color.a = 1.0;

		line_vis.color = score_color;

		// line_vis.color = frustum_color.color(score * 64);

		pcl::PointCloud<pcl::PointXYZ> frustum_cloud_start, frustum_cloud_end;

		float rad_h = (M_PI - height_FOV) / 2;

		// for (int row_pix = 0; row_pix < pix_height; row_pix += step)
		for (int i = 0; i < 2; i++)
		{
			float rad_w = (M_PI - width_FOV) / 2;

			// for (int col_pix = 0; col_pix < pix_width; col_pix += step)
			for (int n = 0; n < 2; n++)
			{
				float x1 = min_range * sin(rad_h) * cos(rad_w);
				float z1 = min_range * sin(rad_h) * sin(rad_w);
				float y1 = min_range * cos(rad_h);
				float x2 = max_range * sin(rad_h) * cos(rad_w);
				float z2 = max_range * sin(rad_h) * sin(rad_w);
				float y2 = max_range * cos(rad_h);

				frustum_cloud_start.push_back(pcl::PointXYZ(x1, y1, z1));
				frustum_cloud_end.push_back(pcl::PointXYZ(x2, y2, z2));

				rad_w += width_FOV;
			}
			rad_h += height_FOV;
		}

		pcl_ros::transformPointCloud(frustum_cloud_start, frustum_cloud_start, view_pose);
		pcl_ros::transformPointCloud(frustum_cloud_end, frustum_cloud_end, view_pose);

		for (size_t i = 0; i < 4; i++)
		{
			geometry_msgs::Point p1, p2;
			pcl::PointXYZ start, end;

			start = frustum_cloud_start.at(i);
			end = frustum_cloud_end.at(i);

			p1.x = start.x;
			p1.y = start.y;
			p1.z = start.z;

			p2.x = end.x;
			p2.y = end.y;
			p2.z = end.z;

			line_vis.points.push_back(p1);
			line_vis.points.push_back(p2);
		}

		line_vis.points.push_back(line_vis.points[0]);
		line_vis.points.push_back(line_vis.points[2]);

		line_vis.points.push_back(line_vis.points[2]);
		line_vis.points.push_back(line_vis.points[6]);

		line_vis.points.push_back(line_vis.points[4]);
		line_vis.points.push_back(line_vis.points[6]);

		line_vis.points.push_back(line_vis.points[0]);
		line_vis.points.push_back(line_vis.points[4]);

		line_vis.points.push_back(line_vis.points[1]);
		line_vis.points.push_back(line_vis.points[3]);

		line_vis.points.push_back(line_vis.points[3]);
		line_vis.points.push_back(line_vis.points[7]);

		line_vis.points.push_back(line_vis.points[5]);
		line_vis.points.push_back(line_vis.points[7]);

		line_vis.points.push_back(line_vis.points[1]);
		line_vis.points.push_back(line_vis.points[5]);

		return line_vis;
	}

	visualization_msgs::Marker textVis(std::string frame_id)
	{
		using namespace std;

		// if (ray_points_list.size() <= 0)
		// {
		//     this->evalPose();
		// }

		visualization_msgs::Marker text;

		text.header.frame_id = frame_id;
		text.header.stamp = ros::Time::now();

		text.id = 0;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text.action = visualization_msgs::Marker::ADD;

		tf::Vector3 origin = view_pose.getOrigin();

		text.scale.z = 0.05;
		text.color.r = 0.0;
		text.color.g = 0.0;
		text.color.b = 0.0;
		text.color.a = 1.0;
		text.pose.position.x = origin.getX() - 0.1;
		text.pose.position.y = origin.getY() - 0.1;
		text.pose.position.z = origin.getZ() - 0.1;
		text.pose.orientation.x = 0.0;
		text.pose.orientation.y = 0.0;
		text.pose.orientation.z = 0.0;
		text.pose.orientation.w = 1.0;

		text.text = "Score: " + to_string(score);

		return text;
	}
};