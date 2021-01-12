#include "ros/ros.h"

#include "tvc_simulator/State.h"

#include <visualization_msgs/Marker.h>

#include <sstream>
#include <string>
#include <time.h>

// global variable with last received rocket state
tvc_simulator::State current_state;

void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}


int main(int argc, char **argv)
{
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

	// Create RViz publisher (10Hz)
	ros::Rate r(10);
	ros::Publisher viz_pub = n.advertise<visualization_msgs::Marker>("rocket_visualization", 1);


  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "rocket_shape";
    marker.id = 0;

    // Set our rocket shape type to be a cylinder
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://tvc_simulator/rviz/Bellalui_1.stl";

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    /*marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;*/
		marker.pose.orientation = current_state.pose.orientation;
    //marker.pose.position = current_state.pose.position;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1e-3;
    marker.scale.y = 1e-3;
    marker.scale.z = 1e-3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (viz_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    viz_pub.publish(marker);
		ros::spinOnce();

		r.sleep();
	}
}
