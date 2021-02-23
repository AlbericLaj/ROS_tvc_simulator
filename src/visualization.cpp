#include "ros/ros.h"

#include "tvc_simulator/State.h"
#include "tvc_simulator/Control.h"

#include <visualization_msgs/Marker.h>

#include <sstream>
#include <string>
#include <time.h>
#include <typeinfo>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// global variable with last received rocket state
tvc_simulator::State current_state;

void rocket_stateCallback(const tvc_simulator::State::ConstPtr &rocket_state) {
    current_state.pose = rocket_state->pose;
    current_state.twist = rocket_state->twist;
    current_state.propeller_mass = rocket_state->propeller_mass;
}

// global variable with last received rocket control
tvc_simulator::Control current_control;

void controlCallback(const tvc_simulator::Control::ConstPtr &control) {
    current_control.torque = control->torque;
    current_control.force = control->force;
}

int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "visualization");
    ros::NodeHandle n;

    // Initialize state
    current_state.pose.orientation.x = 0;
    current_state.pose.orientation.y = 0;
    current_state.pose.orientation.z = 0;
    current_state.pose.orientation.w = 1;


    // Subscribe to state message
    ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 1000, rocket_stateCallback);
    ros::Subscriber control_sub = n.subscribe("control_pub", 1000, controlCallback);

    // Create RViz publisher (10Hz)
    ros::Rate r(10);
    ros::Publisher viz_pub = n.advertise<visualization_msgs::Marker>("rocket_visualization", 5);


    // Moving frame
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "body_frame";

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 100.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();


    visualization_msgs::Marker rocket_marker;

    rocket_marker.header.frame_id = "world";
    // Set the body action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    rocket_marker.action = visualization_msgs::Marker::ADD;
    // Set the namespace and id for this body.  This serves to create a unique ID
    // Any body sent with the same namespace and id will overwrite the old one
    rocket_marker.ns = "rocket_shape";
    rocket_marker.id = 1;

    // Set our rocket shape type to be a mesh
    rocket_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    rocket_marker.mesh_resource = "package://tvc_simulator/rviz/Bellalui_1.stl";

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    rocket_marker.scale.x = 1e-3;
    rocket_marker.scale.y = 1e-3;
    rocket_marker.scale.z = 1e-3;

    // Set the color -- be sure to set alpha to something non-zero!
    rocket_marker.color.r = 0.0f;
    rocket_marker.color.g = 0.0f;
    rocket_marker.color.b = 1.0f;
    rocket_marker.color.a = 1.0;

    while (ros::ok()) {


        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        rocket_marker.pose.position = current_state.pose.position;
        rocket_marker.pose.orientation = current_state.pose.orientation;

        rocket_marker.header.stamp = ros::Time::now();
        rocket_marker.lifetime = ros::Duration();

        //update the position of the body frame
        transformStamped.transform.translation.x = current_state.pose.position.x;
        transformStamped.transform.translation.y = current_state.pose.position.y;
        transformStamped.transform.translation.z = current_state.pose.position.z;

        transformStamped.header.stamp = ros::Time::now();

        // Publish the marker
        while (viz_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        //
        tfb.sendTransform(transformStamped);
        viz_pub.publish(rocket_marker);
        ros::spinOnce();

        r.sleep();
    }
}
