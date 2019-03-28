#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cmath>

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

  view_origin.setX(radius * sin(theta) * cos(psi));
  view_origin.setY(radius * sin(theta) * sin(psi));
  view_origin.setZ(radius * cos(theta));

  z_direction.setX(observation_center.getX() - view_origin.getX());
  z_direction.setY(observation_center.getY() - view_origin.getY());
  z_direction.setZ(observation_center.getZ() - view_origin.getZ());

  z_direction.normalize();

  // random vector just to find one that's perpendicular to z_direction
  rand_vector.setX((double)rand() / RAND_MAX);
  rand_vector.setY((double)rand() / RAND_MAX);
  rand_vector.setZ((double)rand() / RAND_MAX);
  rand_vector.normalize();

  // TODO: y needs to point down

  y_direction = z_direction.cross(rand_vector);
  y_direction.normalize();

  x_direction = z_direction.cross(y_direction);
  x_direction.normalize();

  rotation_matrix.setValue(x_direction.getX(), x_direction.getY(), x_direction.getZ(), y_direction.getX(),
                           y_direction.getY(), y_direction.getZ(), z_direction.getX(), z_direction.getY(),
                           z_direction.getZ());

  rotation_matrix.getRotation(view_orientation);

  view_pose.setOrigin(view_origin);
  view_pose.setRotation(view_orientation);

  return view_pose;
}

// geometry_msgs::PoseArray genPoses(int num_poses, float scale)
// {
//   float x_max = 1, x_min = 0;
//   float y_max = 1, y_min = 0;
//   float z_max = 1, z_min = 0;

//   // float x_range, y_range, z_range;
//   float x_center, y_center, z_center;
//   float x_side, y_side, z_side;

//   ros::param::get("~x_max", x_max);
//   ros::param::get("~x_min", x_min);

//   ros::param::get("~y_max", y_max);
//   ros::param::get("~y_min", y_min);

//   ros::param::get("~z_max", z_max);
//   ros::param::get("~z_min", z_min);

//   // x_range = x_max - x_min;
//   // y_range = y_max - y_min;
//   // z_range = z_max - z_min;

//   x_center = (x_max + x_min) / 2;
//   y_center = (y_max + y_min) / 2;
//   z_center = (z_max + z_min) / 2;

//   x_side = sqrt((x_min - x_max) * (x_min - x_max)) * scale;
//   y_side = sqrt((y_min - y_max) * (y_min - y_max)) * scale;
//   z_side = sqrt((z_min - z_max) * (z_min - z_max)) * scale;

//   float x_coords[num_poses];
//   float y_coords[num_poses];
//   float z_coords[num_poses];

//   float R_orient[num_poses];
//   float P_orient[num_poses];
//   float Y_orient[num_poses];

//   float rand_x, rand_y, rand_z;
//   float rand_R, rand_P, rand_Y;
//   float roll, pitch, yaw;

//   srand(time(NULL));

//   for (int i = 0; i < num_poses; i++)
//   {
//     // rand_x = ((double)rand() / RAND_MAX) * x_range + x_min;
//     // rand_y = ((double)rand() / RAND_MAX) * y_range + y_min;
//     // rand_z = ((double)rand() / RAND_MAX) * z_range + z_min;

//     rand_x = ((double)rand() / RAND_MAX) * x_side + (x_center - (x_side / 2));
//     rand_y = ((double)rand() / RAND_MAX) * y_side + (y_center - (y_side / 2));
//     rand_z = ((double)rand() / RAND_MAX) * z_side + (z_center - (z_side / 2));

//     // std::cout << rand_x << " " << rand_y << " " << rand_z << std::endl;

//     rand_R = ((double)rand() / RAND_MAX) * (2 * M_PI) - M_PI;
//     rand_P = ((double)rand() / RAND_MAX) * (2 * M_PI) - M_PI;
//     rand_Y = ((double)rand() / RAND_MAX) * (2 * M_PI) - M_PI;

//     // rand_x *= scale;
//     // rand_y *= scale;
//     // rand_z *= scale;

//     x_coords[i] = rand_x;
//     y_coords[i] = rand_y;
//     z_coords[i] = rand_z;

//     R_orient[i] = rand_R;
//     P_orient[i] = rand_P;
//     Y_orient[i] = rand_Y;
//   }

//   geometry_msgs::PoseArray all_poses;
//   tf::Quaternion Q_tf;
//   geometry_msgs::Quaternion Q_orient;

//   all_poses.header.frame_id = "/base_link";
//   all_poses.header.stamp = ros::Time::now();

//   for (int i = 0; i < num_poses; i++)
//   {
//     geometry_msgs::Pose one_pose;

//     one_pose.position.x = x_coords[i];
//     one_pose.position.y = y_coords[i];
//     one_pose.position.z = z_coords[i];

//     roll = R_orient[i];
//     pitch = P_orient[i];
//     yaw = Y_orient[i];

//     Q_tf.setRPY(roll, pitch, yaw);
//     tf::quaternionTFToMsg(Q_tf, Q_orient);

//     one_pose.orientation.x = Q_orient.x;
//     one_pose.orientation.y = Q_orient.y;
//     one_pose.orientation.z = Q_orient.z;
//     one_pose.orientation.w = Q_orient.w;

//     all_poses.poses.push_back(one_pose);
//   }

//   return all_poses;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_publisher");

  ros::NodeHandle n;

  ros::Publisher pub_poseArray = n.advertise<geometry_msgs::PoseArray>("pose_array", 10);

  ros::Rate loop_rate(10);

  srand(time(NULL));

  int n_poses = 20;
  float r_min = 0, r_max = 1;

  std::string fixed_frame = "/map";
  geometry_msgs::PoseArray all_poses;
  geometry_msgs::Pose pose_to_push;
  tf::Point origin;
  tf::Pose one_pose;

  ros::param::get("~n_poses", n_poses);
  ros::param::get("~r_min", r_min);
  ros::param::get("~r_max", r_max);
  ros::param::get("~fixed_frame", fixed_frame);

  origin.setX(0);
  origin.setY(0);
  origin.setZ(0);

  all_poses.header.frame_id = fixed_frame;
  all_poses.header.stamp = ros::Time::now();

  for (int i = 0; i < n_poses; i++)
  {
    one_pose = genPose(r_min, r_max, origin);

    tf::poseTFToMsg(one_pose, pose_to_push);

    all_poses.poses.push_back(pose_to_push);
  }

  while (ros::ok())
  {
    pub_poseArray.publish(all_poses);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
