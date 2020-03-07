#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <smobex_actions/smobex_explorerAction.h>

#include <smobex_actions/smobex.h>

class SmobexExplorer
{
private:

    ros::NodeHandle _nh;
    actionlib::SimpleActionServer<smobex_actions::smobex_explorerAction> _as;
    std::string _action_name;
    smobex_actions::smobex_explorerActionFeedback _feedback;
    smobex_actions::smobex_explorerActionResult _result;    

public:

    SmobexExplorer(std::string name) : _as(_nh, name, boost::bind(&SmobexExplorer::executeCB, this, _1), false), _action_name(name)
    {
        _as.start();
    }

    ~SmobexExplorer(void)
    {
    }

    void executeCB(const smobex_actions::smobex_explorerGoalConstPtr &goal)
    {

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smober_actions_server");

    SmobexExplorer action_explorer("action_explorer");
    ros::spin();

    return 0;
}