#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
import visualization_msgs.msg
import std_msgs.msg

if __name__ == '__main__':

    rospy.init_node('path_draw')

    listener = tf.TransformListener()

    pub = rospy.Publisher('path_drawn', visualization_msgs.msg.Marker, queue_size=10)

    rate = rospy.Rate(10.0)

    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = "/base_link"
    # marker.type = marker.LINE_STRIP
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.2
    marker.color.g = 0.2
    marker.color.b = 0.2

     # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []

    while not rospy.is_shutdown():

        try:
            (trans, rot) = listener.lookupTransform(
                '/base_link', '/camera_depth_optical_frame', rospy.Time(0))

            line_point = geometry_msgs.msg.Point()

            line_point.x = trans[0]
            line_point.y = trans[1]
            line_point.z = trans[2]
            
            marker.points.append(line_point)

            # Publish the Marker
            pub.publish(marker)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('No tf')
            continue 

        
        # # first point
        # first_line_point = geometry_msgs.msg.Point()
        # first_line_point.x = 0.0
        # first_line_point.y = 0.0
        # first_line_point.z = 0.0
        # marker.points.append(first_line_point)
        # # second point
        # second_line_point = geometry_msgs.msg.Point()
        # second_line_point.x = 1.0
        # second_line_point.y = 1.0
        # second_line_point.z = 0.0
        # marker.points.append(second_line_point)

        

        rate.sleep()
