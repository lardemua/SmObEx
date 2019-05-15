#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <smobex_explorer_action_skill_server/smobex_explorer_action_skill_server.h>

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
