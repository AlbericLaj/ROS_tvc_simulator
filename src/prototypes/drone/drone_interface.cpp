#include "ros/ros.h"

#include "tvc_simulator/DroneControl.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/DroneState.h"
#include "tvc_simulator/Control.h"

#include "../../../submodule/polympc/src/utils/helpers.hpp"

using namespace Eigen;

ros::Publisher control_pub, drone_state_pub;

float CM_to_thrust_distance = 0.4;


Matrix<double, 2, 2> sysA;
Matrix<double, 2, 1> sysB;
Matrix<double, 1, 2> sysC;
Matrix<double, 2, 1> x_servo1;
Matrix<double, 2, 1> x_servo2;

void publishConvertedControl(const tvc_simulator::DroneControl::ConstPtr &drone_control) {

    float thrust = drone_control->top + drone_control->bottom;
    float torque = drone_control->top - drone_control->bottom;

    Eigen::Vector3d thrust_vertical(0, 0, thrust);

    // ss model for ts = 0.06//TODO use integrator time step instead
    x_servo1 = sysA * x_servo1 + sysB * ((double) drone_control->servo1);
    double servo1 = sysC * x_servo1;

    x_servo2 = sysA * x_servo2 + sysB * ((double) drone_control->servo2);
    double servo2 = sysC * x_servo2;

    //quaternion representing the rotation of the servos around the Y-axis followed by the rotation around the X-axis
    Eigen::Quaterniond
            thrust_rotation(
            AngleAxisd(servo1, Vector3d::UnitX()) *
            AngleAxisd(servo2, Vector3d::UnitY())
    );

    //rotated thrust vector, in body frame
    Eigen::Vector3d thrust_vector = thrust_rotation._transformVector(thrust_vertical);


    //compute the force and torque at the center of mass and publish them
    tvc_simulator::Control converted_control;

    converted_control.torque.x = thrust_vector.x() * CM_to_thrust_distance;
    converted_control.torque.y = thrust_vector.y() * CM_to_thrust_distance;
    converted_control.torque.z = torque;

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
    //init state system
    x_servo1 << 0, 0;
    x_servo2 << 0, 0;
    sysA << 0.9796, -0.6677,
            0.5, 0;
    sysB << 0.5,
            0;
    sysC << 0.4177, 0.5771;

    // Init ROS time keeper node
    ros::init(argc, argv, "drone_interface");
    ros::NodeHandle n;

    n.getParam("/rocket/CM_to_thrust_distance", CM_to_thrust_distance);

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
