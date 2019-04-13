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

class evaluatePose : public generatePose
{
  public:
    int score = 0;
    int step;
    float min_FOV;
    float width_FOV;
    float height_FOV;
    octomap::OcTree *octree = NULL;
    octomap::OcTree *unknown_octree = NULL;
    octomap::point3d_list ray_points_list;
    octomap::KeySet first_keys;
    octomap::KeySet posterior_keys;

    evaluatePose(int _step, float _min_FOV, float _width_FOV, float _height_FOV)
    {
        step = _step;
        min_FOV = _min_FOV;
        width_FOV = _width_FOV;
        height_FOV = _height_FOV;
    }

    void writeKnownOctomapCallback(const octomap_msgs::OctomapConstPtr &map)
    {
        using namespace octomap;

        // ROS_INFO("I'm inside the callback");

        AbstractOcTree *tree = NULL;

        if (octree != NULL)
        {
            // ROS_INFO("DEBUG: Going to DEL OCTREE");
            delete (octree);
            // ROS_INFO("DEBUG: Octree DEL");
        }

        tree = msgToMap(*map);
        octree = dynamic_cast<OcTree *>(tree);
    }

    void writeUnknownOctomapCallback(const octomap_msgs::OctomapConstPtr &map)
    {
        using namespace octomap;

        // ROS_INFO("I'm inside the callback");

        AbstractOcTree *tree = NULL;

        if (unknown_octree != NULL)
        {
            // ROS_INFO("DEBUG: Going to DEL OCTREE");
            delete (unknown_octree);
            // ROS_INFO("DEBUG: Octree DEL");
        }

        tree = msgToMap(*map);
        unknown_octree = dynamic_cast<OcTree *>(tree);
    }

    void evalPose()
    {
        using namespace octomap;
        using namespace octomath;

        Vector3 origin, start_point, direction, end_point;
        KeyRay ray_keys;

        ros::NodeHandle n;
        sensor_msgs::CameraInfoConstPtr CamInfo;

        int pix_width = 640;
        int pix_height = 480;

        CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info", n,
                                                                      ros::Duration(10));

        ros::Subscriber octomapFull_sub = n.subscribe("/octomap_full", 10, &evaluatePose::writeKnownOctomapCallback, this);
        ros::Subscriber unknownFullMap_sub =
            n.subscribe("/unknown_full_map", 10, &evaluatePose::writeUnknownOctomapCallback, this);

        while (octree == NULL || unknown_octree == NULL)
        {
            ros::spinOnce();

            ros::Duration(0.01).sleep();
        }

        Pose6D octo_pose = poseTfToOctomap(view_pose);

        origin.x() = octo_pose.x();
        origin.y() = octo_pose.y();
        origin.z() = octo_pose.z();

        pix_width = CamInfo->width;
        pix_height = CamInfo->height;

        float delta_rad_w = width_FOV / pix_width;
        float delta_rad_h = height_FOV / pix_height;

        float corner_rad_w = width_FOV / 2;
        float corner_rad_h = height_FOV / 2;

        pcl::PointCloud<pcl::PointXYZ> rays_point_cloud;

        float rad_h = -corner_rad_h;

        for (int row_pix = 0; row_pix < pix_height; row_pix += step)
        {
            float rad_w = -corner_rad_w;

            for (int col_pix = 0; col_pix < pix_width; col_pix += step)
            {
                float x = 1 * sin(rad_w) * cos(rad_h);
                float y = 1 * sin(rad_w) * sin(rad_h);
                float z = 1 * cos(rad_w);

                rays_point_cloud.push_back(pcl::PointXYZ(x, y, z));

                rad_w += delta_rad_w * step;
            }
            rad_h += delta_rad_h * step;
        }

        // rays_point_cloud.push_back(pcl::PointXYZ(-0.01, 0, 0.8));
        // rays_point_cloud.push_back(pcl::PointXYZ(0.01, 0, 0.8));
        pcl_ros::transformPointCloud(rays_point_cloud, rays_point_cloud, view_pose);

        int n_start_points = rays_point_cloud.height * rays_point_cloud.width;
        KeySet oool;
        if (octree != NULL && unknown_octree != NULL)
        {
            for (size_t i = 0; i < n_start_points; i++)
            {
                pcl::PointXYZ point = rays_point_cloud.at(i);

                start_point.x() = point.x;
                start_point.y() = point.y;
                start_point.z() = point.z;

                direction = start_point - origin;

                // octree->castRay(start_point, direction, end_point, true, -1.0);
                octree->castRay(origin, direction, end_point, true, 10);

                start_point = origin + direction.normalized() * min_FOV;

                if (origin.distance(start_point) < origin.distance(end_point))
                {
                    unknown_octree->computeRayKeys(start_point, end_point, ray_keys);

                    // first_keys.insert(*ray_keys.begin());
                    bool first = true;
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
                            oool.insert(*it);
                        }
                    }
                    ray_points_list.push_back(start_point);
                    ray_points_list.push_back(end_point);
                }
            }

            for (KeySet::iterator it = posterior_keys.begin(); it != posterior_keys.end(); it++)
            {
                if (first_keys.find(*it) != first_keys.end())
                {
                    posterior_keys.erase(*it);
                }
            }

            ROS_INFO("first_keys: %lu", first_keys.size());
            ROS_INFO("posterior_keys: %lu", posterior_keys.size());

            ROS_INFO("total_keys: %lu", oool.size());
        }
        else
        {
            ROS_INFO("No OcTree.");
        }
    }

    void getScore()
    {
        score = first_keys.size() + posterior_keys.size();
        ROS_INFO("Pose score: %d", score);
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

    visualization_msgs::MarkerArray discoveredBoxes(std::string frame_id)
    {
        using namespace octomap;

        visualization_msgs::MarkerArray all_boxes;
        double unknown_octree_depth = unknown_octree->getTreeDepth();
        all_boxes.markers.resize(unknown_octree_depth + 1);
        std_msgs::ColorRGBA blue, light_blue;
        ros::Time t = ros::Time::now();

        blue.r = 0.0;
        blue.g = 0.0;
        blue.b = 1.0;
        blue.a = 1.0;

        light_blue.r = 0.0;
        light_blue.g = 0.7;
        light_blue.b = 1.0;
        light_blue.a = 1.0;

        for (OcTree::iterator it = unknown_octree->begin(), end = unknown_octree->end(); it != end;
             ++it)
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
                all_boxes.markers[idx].colors.push_back(blue);
                all_boxes.markers[idx].ns = "first_boxes";
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
                all_boxes.markers[idx].colors.push_back(light_blue);
                all_boxes.markers[idx].ns = "posterior_boxes";
            }
        }

        for (unsigned i = 0; i < all_boxes.markers.size(); i++)
        {
            double size = unknown_octree->getNodeSize(i);

            all_boxes.markers[i].header.frame_id = frame_id;
            all_boxes.markers[i].header.stamp = t;
            // all_boxes.markers[i].ns = "found_boxes";
            all_boxes.markers[i].id = i;
            all_boxes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            all_boxes.markers[i].scale.x = size;
            all_boxes.markers[i].scale.y = size;
            all_boxes.markers[i].scale.z = size;
            // all_boxes.markers[i].color = blue;

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
};
