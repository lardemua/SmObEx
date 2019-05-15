#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <smobex_explorer_action_skill_server/smobex_explorer_action_skill_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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

#include <smobex_explorer/explorer.h>

SmobexExplorerActionSkill::SmobexExplorerActionSkill(std::string name) : as_(nh_, name, boost::bind(&SmobexExplorerActionSkill::executeCB, this, _1), false),
                                                                         action_name_(name)
{
  as_.start();
}

SmobexExplorerActionSkill::~SmobexExplorerActionSkill()
{
}

void SmobexExplorerActionSkill::executeCB(const smobex_explorer_action_skill_msgs::SmobexExplorerActionSkillGoalConstPtr &goal)
{
  ROS_INFO_STREAM("Threshold: " << goal->threshold);
  this->set_succeeded();
}

void SmobexExplorerActionSkill::set_succeeded(std::string outcome)
{
  result_.percentage = 100;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Succeeded";
  result_.outcome = outcome;
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  as_.setSucceeded(result_);
}

void SmobexExplorerActionSkill::set_aborted(std::string outcome)
{
  result_.percentage = 0;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Aborted";
  result_.outcome = outcome;
  ROS_INFO("%s: Aborted", action_name_.c_str());
  as_.setAborted(result_);
}
void SmobexExplorerActionSkill::feedback(float percentage)
{
  feedback_.percentage = percentage;
  feedback_.skillStatus = action_name_.c_str();
  feedback_.skillStatus += " Executing";
  ROS_INFO("%s: Executing. Percentage: %f%%.", action_name_.c_str(), percentage);
  as_.publishFeedback(feedback_);
}

bool SmobexExplorerActionSkill::check_preemption()
{
  if (as_.isPreemptRequested() || !ros::ok())
  {
    result_.percentage = 0;
    result_.skillStatus = action_name_.c_str();
    result_.skillStatus += ": Preempted";
    result_.outcome = "preempted";
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted(result_);
    return true;
  }
  else
  {
    return false;
  }
}
