#!/usr/bin/env python
import rospy
import os

x_max = rospy.get_param("/point_cloud_filter/x_max")
x_min = rospy.get_param("/point_cloud_filter/x_min")

y_max = rospy.get_param("/point_cloud_filter/y_max")
y_min = rospy.get_param("/point_cloud_filter/y_min")

z_max = rospy.get_param("/point_cloud_filter/z_max")
z_min = rospy.get_param("/point_cloud_filter/z_min")

rospy.set_param("/pcl_filters/psx/filter_limit_max",x_max)
rospy.set_param("/pcl_filters/psx/filter_limit_min",x_min)

rospy.set_param("/pcl_filters/psy/filter_limit_max",y_max)
rospy.set_param("/pcl_filters/psy/filter_limit_min",y_min)

rospy.set_param("/pcl_filters/psz/filter_limit_max",z_max)
rospy.set_param("/pcl_filters/psz/filter_limit_min",z_min)

_path=rospy.get_param("/params_path")
cmd = "rosparam dump " + _path + "/params/" + "pcl_params.yaml /point_cloud_filter -v"

os.system(cmd)
