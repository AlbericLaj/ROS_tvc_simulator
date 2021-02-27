#include "ros/ros.h"

#include "tvc_simulator/DroneControl.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/DroneState.h"
#include "tvc_simulator/Control.h"

#include "utils/helpers.hpp"

using namespace Eigen;


ros::Publisher control_pub, drone_state_pub;

void publishConvertedControl(const tvc_simulator::DroneControl::ConstPtr &drone_control) {
    Eigen::Vector3d thrust_minus_z(0, 0, drone_control->thrust);

    //quaternion representing the rotation of the servos around the Y-axis following by the rotation around the X-axis
    Eigen::Quaterniond thrust_rotation(
            AngleAxisd(drone_control->servo1, Vector3d::UnitX()) *
            AngleAxisd(drone_control->servo2, Vector3d::UnitY())
    );

    //rotated thrust vector, in body frame
    Eigen::Vector3d thrust_vector = thrust_rotation._transformVector(thrust_minus_z);


    //compute the force and torque and the center of mass and publish them
    tvc_simulator::Control converted_control;

    converted_control.torque.x = thrust_vector.x() * 0.4;
    converted_control.torque.y = thrust_vector.y() * 0.4;
    converted_control.torque.z = drone_control->torque;

    converted_control.force.x = thrust_vector.x();
    converted_control.force.y = thrust_vector.y();
    converted_control.force.z = thrust_vector.z();

    control_pub.publish(converted_control);
}

void publishConvertedState(const tvc_simulator::State::ConstPtr &rocket_state) {
    tvc_simulator::DroneState converted_state;

    converted_state.twist = rocket_state->twist;
    converted_state.pose = rocket_state->pose;

    drone_state_pub.publish(converted_state);
}

int main(int argc, char **argv) {
    // Init ROS time keeper node
    ros::init(argc, argv, "drone_interface");
    ros::NodeHandle n;

    // Subscribe to drone control
    ros::Subscriber drone_control_sub = n.subscribe("drone_control_pub", 10, publishConvertedControl);

    // Subscribe to rocket state
    ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 10, publishConvertedState);

    // Create control publisher
    control_pub = n.advertise<tvc_simulator::Control>("control_pub", 10);

    // Create control publisher
    drone_state_pub = n.advertise<tvc_simulator::DroneState>("drone_state", 10);

    // Automatic callback of service and publisher from here
    ros::spin();
}
