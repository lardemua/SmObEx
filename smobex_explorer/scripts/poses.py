#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
import visualization_msgs.msg
import tf


def pose_gen():

    # Node params

    pub_poseArray = rospy.Publisher(
        "pose_array", geometry_msgs.msg.PoseArray, queue_size=10)

    rospy.init_node("pose_publisher", anonymous=True)
    rate = rospy.Rate(10)

    # Get the Bounding Box limits
    x_max = rospy.get_param("/pcl_filters/psx/filter_limit_max", 1)
    x_min = rospy.get_param("/pcl_filters/psx/filter_limit_min", 0)

    y_max = rospy.get_param("/pcl_filters/psy/filter_limit_max", 1)
    y_min = rospy.get_param("/pcl_filters/psy/filter_limit_min", 0)

    z_max = rospy.get_param("/pcl_filters/psz/filter_limit_max", 1)
    z_min = rospy.get_param("/pcl_filters/psz/filter_limit_min", 0)

    # Poses
    n_pos = rospy.get_param("~n_poses", 20)
    scale = rospy.get_param("~scale", 1)
    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min

    # Poses Points
    x_coords = np.random.random([n_pos, 1]) * x_range + x_min
    y_coords = np.random.random([n_pos, 1]) * y_range + y_min
    z_coords = np.random.random([n_pos, 1]) * z_range + z_min

    x_coords = x_coords * scale
    y_coords = y_coords * scale
    z_coords = z_coords * scale

    # Poses RPY
    roll_coords = np.random.random([n_pos, 1]) * 2*np.pi - np.pi
    pitch_coords = np.random.random([n_pos, 1]) * 2*np.pi - np.pi
    yaw_coords = np.random.random([n_pos, 1]) * 2*np.pi - np.pi

    all_pose = geometry_msgs.msg.PoseArray()

    all_pose.header.frame_id = "/base_link"
    all_pose.header.stamp = rospy.Time.now()

    for i in range(0, n_pos):

        temp_pose = geometry_msgs.msg.Pose()

        temp_pose.position.x = x_coords[i, 0]
        temp_pose.position.y = y_coords[i, 0]
        temp_pose.position.z = z_coords[i, 0]

        roll = roll_coords[i, 0]
        pitch = pitch_coords[i, 0]
        yaw = yaw_coords[i, 0]

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        temp_pose.orientation.x = quaternion[0]
        temp_pose.orientation.y = quaternion[1]
        temp_pose.orientation.z = quaternion[2]
        temp_pose.orientation.w = quaternion[3]

        all_pose.poses.append(temp_pose)

    while not rospy.is_shutdown():
        # print(pose)
        # pub_pose.publish(pose)
        pub_poseArray.publish(all_pose)
        rate.sleep()


if __name__ == '__main__':

    try:
        pose_gen()
    except rospy.ROSInterruptException:
        pass
