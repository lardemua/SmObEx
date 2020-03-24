#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

        static const std::string planning_group = "manipulator";

        ros::Publisher pub_cloud_clusters = _nh.advertise<sensor_msgs::PointCloud2>("/clusters_cloud", 10);
        ros::Publisher pub_centers_clusters = _nh.advertise<sensor_msgs::PointCloud2>("/clusters_centers", 10);
        ros::Publisher pub_arrows = _nh.advertise<visualization_msgs::MarkerArray>("/pose_arrows", 10);
        ros::Publisher pub_space = _nh.advertise<visualization_msgs::MarkerArray>("/discovered_space", 10);

        moveit::planning_interface::MoveGroupInterface move_group(planning_group);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(planning_group);

        std_msgs::ColorRGBA green_color;
        green_color.r = 0.0;
        green_color.g = 1.0;
        green_color.b = 0.0;
        green_color.a = 1.0;

        float min_range = 0;
        float max_range = 1;
        float width_FOV = M_PI;
        float height_FOV = M_PI;
        float resolution = 0.5;
        std::string frame_id = "/world";

        ros::param::get("~" + ros::names::remap("min_range"), min_range);
        ros::param::get("~" + ros::names::remap("max_range"), max_range);
        ros::param::get("~" + ros::names::remap("width_FOV"), width_FOV);
        ros::param::get("~" + ros::names::remap("height_FOV"), height_FOV);
        ros::param::get("~" + ros::names::remap("frame_id"), frame_id);
        // ros::param::get("octomap_resolution", resolution);

        Smobex explorer;

        explorer.setValues(min_range, max_range, width_FOV, height_FOV, resolution);

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smober_actions_server");

    SmobexExplorer action_explorer("action_explorer");
    ros::spin();

    return 0;
}