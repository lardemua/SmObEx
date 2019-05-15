#ifndef SMOBEX_EXPLORER_ACTION_SKILL_SERVER
#define SMOBEX_EXPLORER_ACTION_SKILL_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <smobex_explorer_action_skill_msgs/SmobexExplorerActionSkillAction.h>

class SmobexExplorerActionSkill
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<smobex_explorer_action_skill_msgs::SmobexExplorerActionSkillAction> as_;
  std::string action_name_;
  smobex_explorer_action_skill_msgs::SmobexExplorerActionSkillFeedback feedback_;
  smobex_explorer_action_skill_msgs::SmobexExplorerActionSkillResult result_;

public:
  SmobexExplorerActionSkill(std::string name);
  ~SmobexExplorerActionSkill(void);
  void executeCB(const smobex_explorer_action_skill_msgs::SmobexExplorerActionSkillGoalConstPtr &goal);
  void feedback(float percentage);
  void set_succeeded(std::string outcome = "succeeded");
  void set_aborted(std::string outcome = "aborted");
  bool check_preemption();
};

#endif // SMOBEX_EXPLORER_ACTION_SKILL_SERVER
