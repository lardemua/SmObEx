#include "ros/ros.h"
#include "std_msgs/String.h"
#include <octomap/AbstractOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;
using namespace octomap_msgs;

void chatterCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  OcTree *octree = NULL;
  //AbstractOcTree *tree = NULL;

  //tree = msgToMap(*msg);
  octree = dynamic_cast<OcTree *>(msgToMap(*msg));

  //ROS_INFO_STREAM(sizeof(tree));
  //ROS_INFO_STREAM(octree->memoryUsage());
  cout << octree->memoryUsage() << "\n";
 
  if (octree != NULL)
  {
  delete(octree);
  }  
  
//  if (tree != NULL)
 // {
   //delete(tree);
   //*tree = NULL;
   //free(tree);
   //tree->clear();
  //}
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "listener");

ros::NodeHandle n;

ros::Subscriber sub = n.subscribe("/octomap_full", 1000, chatterCallback);

ros::spin();

return 0;
}



