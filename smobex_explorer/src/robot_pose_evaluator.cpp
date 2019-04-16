#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <tf/transform_listener.h>
#include <smobex_explorer/explorer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;
float marker_pos = 0;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last;

ros::Publisher pub_lines;
ros::Publisher pub_space;
ros::Publisher pub_text;

visualization_msgs::Marker line, text, frustum_lines;
visualization_msgs::MarkerArray single_view_boxes;

// tf::TransformListener *listener;

void clickCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Evaluating pose...");

    // tf::StampedTransform transform;

    // try
    // {
    //     listener->lookupTransform("/base_link", "/camera_depth_optical_frame",
    //                               ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //     // ros::Duration(1.0).sleep();
    //     return;
    // }

    ros::NodeHandle n;

    visualization_msgs::InteractiveMarkerInitConstPtr cam_feedback;

    cam_feedback = ros::topic::waitForMessage<visualization_msgs::InteractiveMarkerInit>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full", n, ros::Duration(10));

    ROS_INFO_STREAM("Pose received...");

    evaluatePose pose(20, 0.8, 10, 58 * M_PI / 180, 45 * M_PI / 180);
    // pose.view_pose = transform;
    tf::poseMsgToTF(cam_feedback->markers[0].pose, pose.view_pose);

    pose.evalPose();

    line = pose.rayLinesVis("base_link");
    frustum_lines = pose.frustumLinesVis("base_link");
    text = pose.textVis("base_link");
    single_view_boxes = pose.discoveredBoxesVis("base_link");

    pub_lines.publish(line);
    pub_lines.publish(frustum_lines);
    pub_space.publish(single_view_boxes);
    pub_text.publish(text);

    ros::spinOnce();

    ROS_INFO_STREAM("Evaluating pose...");
}

Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

InteractiveMarker makeEmptyMarker(bool dummyBox = true)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.pose.position.y = -3.0 * marker_pos++;
    ;
    int_marker.scale = 1;

    return int_marker;
}

void makeMenuMarker(std::string name)
{
    InteractiveMarker int_marker = makeEmptyMarker();
    int_marker.name = name;

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
}

void initMenu()
{
    h_first_entry = menu_handler.insert("Evaluate Pose", &clickCB);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "menu");

    ros::NodeHandle n;

    // tf::TransformListener lr(ros::Duration(10));
    // listener = &lr;

    pub_lines = n.advertise<visualization_msgs::Marker>("/ray_cast_lines", 10);
    pub_space = n.advertise<visualization_msgs::MarkerArray>("/discovered_space", 10);
    pub_text = n.advertise<visualization_msgs::Marker>("/pose_text", 10);

    server.reset(new InteractiveMarkerServer("menu", "", false));

    initMenu();

    makeMenuMarker("marker1");

    menu_handler.apply(*server, "marker1");
    server->applyChanges();

    ros::spin();

    server.reset();
}
