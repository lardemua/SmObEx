#include <smobex_explorer_action_skill_server/smobex_explorer_action_skill_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smobex_explorer_action_skill");
  ros::NodeHandle nh("~");
  std::string skill_name;
  nh.param<std::string>("SkillName", skill_name, "SmobexExplorerActionSkill");
  SmobexExplorerActionSkill smobex_explorer_action(skill_name);
  // ros::spin();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
