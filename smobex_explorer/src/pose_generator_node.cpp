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

  //   origin.setX(0);
  //   origin.setY(0);
  //   origin.setZ(0);
  origin.setZero();

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
