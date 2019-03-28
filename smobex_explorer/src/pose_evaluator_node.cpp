#include <geometry_msgs/PoseArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

using namespace std;
using namespace octomap;

class evalPose
{
public:
  geometry_msgs::PoseArray all_poses;
  AbstractOcTree* tree;
  AbstractOccupancyOcTree* octree = NULL;

  void getPoses(const geometry_msgs::PoseArrayConstPtr& poses_rcvd)
  {
    all_poses = *poses_rcvd.get();
  }

  void evalAllPoses(const octomap_msgs::Octomap& map_rcvd)
  {
    tree = octomap_msgs::fullMsgToMap(map_rcvd);

    if (tree)
    {
      octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
    }

    double ProbHit = octree->getProbHit();

    cout << ProbHit;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_evaluator");

  ros::NodeHandle n;

  evalPose evaluate;

  // ros::Subscriber pose_sub = n.subscribe("pose_array", 10, evalPose);
  ros::Subscriber pose_sub = n.subscribe("pose_array", 10, &evalPose::getPoses, &evaluate);
  ros::Subscriber octomap_sub = n.subscribe("octomap_full", 10, &evalPose::evalAllPoses, &evaluate);

  ros::spin();

  return 0;
}
