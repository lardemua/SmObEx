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

class explorePose
{
  private:
    /* data */
  public:
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

        return view_pose;
    }
};
