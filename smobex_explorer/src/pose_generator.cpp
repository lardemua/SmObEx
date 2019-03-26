#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_publisher");

	ros::NodeHandle n;

	ros::Publisher pub_poseArray = n.advertise<geometry_msgs::PoseArray>("pose_array",10);

	ros::Rate loop_rate(10);

	int n_poses=20;

	float x_max = 1, x_min = 0;
	float y_max = 1, y_min = 0;
	float z_max = 1, z_min = 0;

	float x_range, y_range, z_range;

	float scale=1;

	ros::param::get("/pcl_filters/psx/filter_limit_max", x_max);
	ros::param::get("/pcl_filters/psx/filter_limit_min", x_min);

	ros::param::get("/pcl_filters/psy/filter_limit_max", y_max);
	ros::param::get("/pcl_filters/psy/filter_limit_min", y_min);

	ros::param::get("/pcl_filters/psz/filter_limit_max", z_max);
	ros::param::get("/pcl_filters/psz/filter_limit_min", z_min);

	ros::param::get("~scale", scale);
	ros::param::get("~n_poses", n_poses);

	x_range = x_max - x_min;
	y_range = y_max - y_min;
	z_range = z_max - z_min;

	float x_coords[n_poses];
	float y_coords[n_poses];
	float z_coords[n_poses];

	float R_orient[n_poses];
	float P_orient[n_poses];
	float Y_orient[n_poses];

	float rand_x, rand_y, rand_z;
	float rand_R, rand_P, rand_Y;
	float roll, pitch, yaw;

	srand(time(NULL)); 

	for(int i = 0; i < n_poses; i++)
	{
		rand_x = ( (double) rand() / RAND_MAX ) * x_range + x_min;
		rand_y = ( (double) rand() / RAND_MAX ) * y_range + y_min;
		rand_z = ( (double) rand() / RAND_MAX ) * z_range + z_min;

		//std::cout << rand_x << " " << rand_y << " " << rand_z << std::endl;

		rand_R = ( (double) rand() / RAND_MAX ) * (2*M_PI) - M_PI;
		rand_P = ( (double) rand() / RAND_MAX ) * (2*M_PI) - M_PI;
		rand_Y = ( (double) rand() / RAND_MAX ) * (2*M_PI) - M_PI;

		rand_x *= scale;
		rand_y *= scale;
		rand_z *= scale;

		x_coords[i] = rand_x;
		y_coords[i] = rand_y;
		z_coords[i] = rand_z;

		R_orient[i] = rand_R;
		P_orient[i] = rand_P;
		Y_orient[i] = rand_Y;
	}
	
	geometry_msgs::PoseArray all_poses;
	tf::Quaternion Q_tf;
	geometry_msgs::Quaternion Q_orient;

	all_poses.header.frame_id="/base_link";
	all_poses.header.stamp=ros::Time::now();

	for(int i = 0; i < n_poses; i++)
	{
		geometry_msgs::Pose one_pose;

		one_pose.position.x = x_coords[i];
		one_pose.position.y = y_coords[i];
		one_pose.position.z = z_coords[i];

		roll = R_orient[i];
		pitch = P_orient[i];
		yaw = Y_orient[i];

		Q_tf.setRPY(roll ,pitch, yaw);
		tf::quaternionTFToMsg(Q_tf, Q_orient);

		one_pose.orientation.x = Q_orient.x;
		one_pose.orientation.y = Q_orient.y;
		one_pose.orientation.z = Q_orient.z;
		one_pose.orientation.w = Q_orient.w;

		all_poses.poses.push_back(one_pose);

	}

	while(ros::ok())
	{

		pub_poseArray.publish(all_poses);

		ros::spinOnce();

		loop_rate.sleep();

	}
	
	return 0;
}


