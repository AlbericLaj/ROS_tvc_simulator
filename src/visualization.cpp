#include "ros/ros.h"

#include "tvc_simulator/State.h"
#include "tvc_simulator/Control.h"
#include "tvc_simulator/Trajectory.h"

#include <visualization_msgs/Marker.h>

#include <sstream>
#include <string>
#include <time.h>
#include <typeinfo>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Path.h"

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

// global variable with the MPC trajectory
tvc_simulator::Trajectory current_mpc_horizon;

void mpcHorizonCallback(const tvc_simulator::Trajectory::ConstPtr &traj) {
    current_mpc_horizon = *traj;
}

// global variable with the MPC trajectory
tvc_simulator::Trajectory current_target_trajectory;

void targetTrajCallback(const tvc_simulator::Trajectory::ConstPtr &traj) {
    current_target_trajectory = *traj;
}

int marker_id_count = 0;

void init_marker(visualization_msgs::Marker &marker, std::string ns, float r, float g, float b, float a = 1.0, std::string frame_id = "world") {
    // set the frame of reference
    marker.header.frame_id = frame_id;

    // Set the body action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the namespace and id for this body.  This serves to create a unique ID
    // Any body sent with the same namespace and id will overwrite the old one
    marker.ns = ns;
    marker.id = marker_id_count;
    marker_id_count++;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
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

    //Initialize control
    current_control.force.x = 0;
    current_control.force.y = 0;
    current_control.force.z = 0;

    //Initialize trajectory
    for (int i = 0; i < 11; i++) {
        geometry_msgs::Point point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        current_mpc_horizon.trajectory.push_back(point);
    }


    // Subscribe to state message
    ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 1000, rocket_stateCallback);
    ros::Subscriber control_sub = n.subscribe("control_pub", 1000, controlCallback);
    ros::Subscriber mpc_horizon_sub = n.subscribe("mpc_horizon", 1000, mpcHorizonCallback);
    ros::Subscriber target_trajectory_sub = n.subscribe("target_trajectory", 1000, targetTrajCallback);

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

    visualization_msgs::Marker rocket_marker, thrust_vector, mpc_horizon, target_trajectory;

    init_marker(rocket_marker, "rocket marker", 0.75, 0.75, 0.75);
    init_marker(thrust_vector, "thrust vector", 1, 0.5, 0);
    init_marker(mpc_horizon, "mpc horizon", 0.1, 0.3, 0.7, 0.4);
    init_marker(target_trajectory, "target trajectory", 0.15, 0.5, 0.25, 0.4);

    //setup rocket marker
    rocket_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string stl_name;
    n.getParam("/visualization/stl_model", stl_name);
    rocket_marker.mesh_resource = "package://tvc_simulator/rviz/"+stl_name;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    float stl_scale;
    n.getParam("/visualization/stl_model_scale", stl_scale);
    rocket_marker.scale.x = stl_scale;
    rocket_marker.scale.y = stl_scale;
    rocket_marker.scale.z = stl_scale;

    //thrust vector
    const float shaft_diameter = 0.1;
    const float arrow_diameter = 0.2;
    const float head_length = 0.2;
    const float thrust_scaling = 0.001;
    //TODO use real offset and correct rocket scaling
    float offset;
    n.getParam("/visualization/CM_to_thrust_distance", offset);

    thrust_vector.type = visualization_msgs::Marker::ARROW;
    thrust_vector.scale.x = shaft_diameter;
    thrust_vector.scale.y = arrow_diameter;
    thrust_vector.scale.z = head_length;

    //MPC horizon
    const float line_width = 0.1;
    mpc_horizon.type = visualization_msgs::Marker::LINE_STRIP;
    mpc_horizon.scale.x = line_width;

    //target trajectory
    target_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    target_trajectory.scale.x = line_width;

    // Create RViz publisher (10Hz)
    ros::Rate r(10);
    ros::Publisher viz_pub = n.advertise<visualization_msgs::Marker>("rocket_visualization", marker_id_count + 1);

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

        //update thrust vector
        thrust_vector.points.resize(2);

        //rotate thrust vector
        tf2::Quaternion q_rot;
        tf2::convert(current_state.pose.orientation, q_rot);

        tf2::Quaternion thrust_vector_body(-current_control.force.x, -current_control.force.y, -current_control.force.z,
                                           0);
        tf2::Quaternion thrust_vector_inertial = q_rot * thrust_vector_body * q_rot.inverse();

        tf2::Quaternion minus_z_body(0, 0, -1, 0);
        tf2::Quaternion minus_z_inertial = q_rot * minus_z_body * q_rot.inverse();

        //start of arrow, offset in the direction -z from the center of mass
        thrust_vector.points[0] = current_state.pose.position;
        thrust_vector.points[0].x += minus_z_inertial.x() * offset;
        thrust_vector.points[0].y += minus_z_inertial.y() * offset;
        thrust_vector.points[0].z += minus_z_inertial.z() * offset;

        //end of arrow
        thrust_vector.points[1] = thrust_vector.points[0];
        thrust_vector.points[1].x += thrust_vector_inertial.x() * thrust_scaling;
        thrust_vector.points[1].y += thrust_vector_inertial.y() * thrust_scaling;
        thrust_vector.points[1].z += thrust_vector_inertial.z() * thrust_scaling;

        thrust_vector.header.stamp = ros::Time::now();
        thrust_vector.lifetime = ros::Duration();

        //MPC horizon
        mpc_horizon.header.stamp = ros::Time::now();
        mpc_horizon.lifetime = ros::Duration();

        mpc_horizon.points.clear();
        for (auto &traj_point : current_mpc_horizon.trajectory) {
            geometry_msgs::Point point;
            point.x = traj_point.x;
            point.y = traj_point.y;
            point.z = traj_point.z;
            mpc_horizon.points.push_back(point);
        }


        //target trajectory
        target_trajectory.header.stamp = ros::Time::now();
        target_trajectory.lifetime = ros::Duration();

        target_trajectory.points.clear();
        for (auto &traj_point : current_target_trajectory.trajectory) {
            geometry_msgs::Point point;
            point.x = traj_point.x;
            point.y = traj_point.y;
            point.z = traj_point.z;
            target_trajectory.points.push_back(point);
        }

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
        viz_pub.publish(thrust_vector);
        viz_pub.publish(mpc_horizon);
        viz_pub.publish(target_trajectory);
        ros::spinOnce();

        r.sleep();
    }
}
