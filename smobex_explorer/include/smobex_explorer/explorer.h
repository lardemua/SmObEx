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

class evaluatePose : public generatePose
{
  public:
    float score = 0;

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

    pcl::PointCloud<pcl::PointXYZ> rays_point_cloud;

    evaluatePose(int _step, float _min_range, float _max_range, float _width_FOV, float _height_FOV)
    {
        step = _step;
        min_range = _min_range;
        max_range = _max_range;
        width_FOV = _width_FOV;
        height_FOV = _height_FOV;

        ros::NodeHandle n;
        sensor_msgs::CameraInfoConstPtr CamInfo; //TODO STATIC

        CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info", n,
                                                                      ros::Duration(10));

        pix_width = CamInfo->width;
        pix_height = CamInfo->height;

        float delta_rad_w = width_FOV / pix_width;
        float delta_rad_h = height_FOV / pix_height;

        float corner_rad_w = width_FOV / 2;
        float corner_rad_h = height_FOV / 2;

        //generate the points of the camera rays
        float rad_h = (M_PI - height_FOV) / 2;

        for (int row_pix = 0; row_pix < pix_height; row_pix += step)
        {
            float rad_w = (M_PI - width_FOV) / 2;

            for (int col_pix = 0; col_pix < pix_width; col_pix += step)
            {
                float x = 1 * sin(rad_h) * cos(rad_w);
                float z = 1 * sin(rad_h) * sin(rad_w);
                float y = 1 * cos(rad_h);

                rays_point_cloud.push_back(pcl::PointXYZ(x, y, z));

                rad_w += delta_rad_w * step;
            }
            rad_h += delta_rad_h * step;
        }

        // rays_point_cloud.push_back(pcl::PointXYZ(-0.01, 0, 0.8));
        // rays_point_cloud.push_back(pcl::PointXYZ(0.01, 0, 0.8));
        // pcl_ros::transformPointCloud(rays_point_cloud, rays_point_cloud, view_pose);
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

        // ROS_INFO("Known map written");
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

        // ROS_INFO("Unknown map written");
    }

    void evalPose()
    {
        using namespace octomap;
        using namespace octomath;

        Vector3 origin, start_point, direction, end_point;
        KeyRay ray_keys;

        ros::NodeHandle n;

        //TODO wait for message
        // ros::Subscriber octomapFull_sub = n.subscribe("/octomap_full", 10, &evaluatePose::writeKnownOctomapCallback, this);
        // ros::Subscriber unknownFullMap_sub =
        //     n.subscribe("/unknown_full_map", 10, &evaluatePose::writeUnknownOctomapCallback, this);

        // while (octree == NULL || unknown_octree == NULL)
        // {
        //     ros::spinOnce();

        //     ros::Duration(0.01).sleep();
        // }

        writeKnownOctomap();
        writeUnknownOctomap();

        Pose6D octo_pose = poseTfToOctomap(view_pose);

        origin.x() = octo_pose.x();
        origin.y() = octo_pose.y();
        origin.z() = octo_pose.z();

        pcl_ros::transformPointCloud(rays_point_cloud, rays_point_cloud, view_pose);

        int n_start_points = rays_point_cloud.height * rays_point_cloud.width;
        if (octree != NULL && unknown_octree != NULL)
        {
//TODO
#pragma omp parallel for
            for (size_t i = 0; i < n_start_points; i++)
            {
                pcl::PointXYZ point = rays_point_cloud.at(i);

                start_point.x() = point.x;
                start_point.y() = point.y;
                start_point.z() = point.z;

                direction = start_point - origin;

                // octree->castRay(start_point, direction, end_point, true, -1.0);
                octree->castRay(origin, direction, end_point, true, max_range);

                start_point = origin + direction.normalized() * min_range;
                octree->getRayIntersection(origin, direction, end_point, end_point);

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

            getScore();

            // ROS_INFO("first_keys: %lu", first_keys.size());
            // ROS_INFO("posterior_keys: %lu", posterior_keys.size());
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

        point3d min, max;

        ros::param::get("x_max", max.x());
        ros::param::get("y_max", max.y());
        ros::param::get("z_max", max.z());

        ros::param::get("x_min", min.x());
        ros::param::get("y_min", min.y());
        ros::param::get("z_min", min.z());

        // ROS_INFO("max: %f %f %f", max.x(), max.y(), max.z());
        // ROS_INFO("min: %f %f %f", min.x(), min.y(), min.z());

        float weight = 0.5;

        float found_volume = (first_keys.size() + posterior_keys.size() * weight) * one_volume;
        // float total_volume = 0;
        // float outside_volume = 0, inside_volume = 0;

        /*
//TODO
#pragma omp parallel for
        for (octomap::OcTree::iterator it = unknown_octree->begin(); it != unknown_octree->end(); it++)
        {
            octomap::OcTreeKey key = it.getKey();
            if (unknown_octree->search(key))
            {
//first formula
#if 0
                float size = it.getSize();
                float volume = size * size * size;

                total_volume += volume;

//second formula
#elif 0
                float size = it.getSize();
                float volume = size * size * size;

                float delta = resolution * 2;

                point3d coord = unknown_octree->keyToCoord(key);
                point3d x_incrm(delta, 0, 0);
                point3d y_incrm(0, delta, 0);
                point3d z_incrm(0, 0, delta);

                OcTreeNode *x_pos, *x_neg, *y_pos, *y_neg, *z_pos, *z_neg;
                x_pos = unknown_octree->search(coord + x_incrm);
                x_neg = unknown_octree->search(coord - x_incrm);
                y_pos = unknown_octree->search(coord + y_incrm);
                y_neg = unknown_octree->search(coord - y_incrm);
                z_pos = unknown_octree->search(coord + z_incrm);
                z_neg = unknown_octree->search(coord - z_incrm);

                if (x_pos && x_neg && y_pos && y_neg && z_pos && z_neg)
                {
                    inside_volume += volume;
                }
                else
                {
                    outside_volume += volume;
                }
#endif
            }
        }
        
        // total_volume = outside_volume + inside_volume * weight;
*/
        //third formula

        point3d deltas = max - min;

        float total_volume = deltas.x() * deltas.y() * deltas.z();
        float inner_volume = (deltas.x() - resolution * 2) * (deltas.y() - resolution * 2) * (deltas.z() - resolution * 2);
        float outer_volume = total_volume - inner_volume;

        float score_volume = outer_volume + inner_volume * weight;

        score = found_volume / score_volume;
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

        class_colormap frustum_color("winter", 64, 1, true);

        line_vis.color = frustum_color.color(score * 64);

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