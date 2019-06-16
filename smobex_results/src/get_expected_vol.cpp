#include "ros/ros.h"
#include "std_msgs/String.h"
#include "octomap_msgs/Octomap.h"
#include "octomap/OcTree.h"
#include "octomap/AbstractOcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"

void getVolume(const octomap_msgs::OctomapConstPtr &map)
{
    octomap::AbstractOcTree *tree = NULL;
    octomap::OcTree *octree = NULL;

    tree = octomap_msgs::msgToMap(*map);
    octree = dynamic_cast<octomap::OcTree *>(tree);

    std::vector<double> volumes;

    for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); it++)
    {
        if (!octree->isNodeOccupied(*it))
        {
            double size = it.getSize();
            double volume = size * size * size;
            volumes.push_back(volume);
        }
    }

    double total_volume = 0;

    for (size_t i = 0; i < volumes.size(); i++)
    {
        total_volume += volumes[i];
    }
    
    ROS_INFO_STREAM("Volume expected: " << total_volume << " m3");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "result_expected_volume");

    ros::NodeHandle n;

    ROS_INFO("Waiting for expected volume");

    ros::Subscriber sub = n.subscribe("/unknown_full_map", 100, getVolume);

    ros::spin();

    return 0;
}
