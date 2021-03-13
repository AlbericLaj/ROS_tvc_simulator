#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Waypoint.h"
#include "tvc_simulator/Trajectory.h"

#include "tvc_simulator/GetFSM.h"
#include "tvc_simulator/GetWaypoint.h"

#include <time.h>

#include <sstream>
#include <string>
#include <vector>

#include "utils/helpers.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <chrono>
#include "std_msgs/String.h"


// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Global variable with last received rocket state
tvc_simulator::State current_state;

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr &rocket_state) {
    current_state.pose = rocket_state->pose;
    current_state.twist = rocket_state->twist;
    current_state.propeller_mass = rocket_state->propeller_mass;
}

Eigen::Matrix<double, 3, 1> current_target_apogee;

// Service function: send back waypoint at requested time
bool sendWaypoint(tvc_simulator::GetWaypoint::Request &req, tvc_simulator::GetWaypoint::Response &res) {
    Eigen::Matrix<double, 3, 1> next_waypoint;
    next_waypoint = current_target_apogee;

    res.target_point.position.x = next_waypoint(0);
    res.target_point.position.y = next_waypoint(1);
    res.target_point.position.z = next_waypoint(2);

    return true;
}

int main(int argc, char **argv) {
    // Init ROS guidance node
    ros::init(argc, argv, "guidance");
    ros::NodeHandle n;


    // Setup Time_keeper client and srv variable for FSM and time synchronization
    ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
    tvc_simulator::GetFSM srv_fsm;

    // Create waypoint service
    ros::ServiceServer waypoint_service = n.advertiseService("getWaypoint", sendWaypoint);

    // Create waypoint trajectory publisher
    ros::Publisher target_trajectory_pub = n.advertise<tvc_simulator::Trajectory>("target_trajectory", 10);

    // Subscribe to state message from simulation
    ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);

    // Initialize fsm
    current_fsm.time_now = 0;
    current_fsm.state_machine = "Idle";

    ros::Publisher coast_pub = n.advertise<std_msgs::String>("commands", 10);

    // Thread to compute guidance. Duration defines interval time in seconds
    ros::Timer guidance_thread = n.createTimer(ros::Duration(0.3), [&](const ros::TimerEvent &) {
        // Get current FSM and time
        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }

        // State machine ------------------------------------------
        if (current_fsm.state_machine.compare("Idle") == 0) {
            // Do nothing
        } else if (current_fsm.state_machine.compare("Launch") == 0) {
            ROS_INFO("%f", current_fsm.time_now);
            if (current_fsm.time_now < 5) current_target_apogee << 5, 0, 10;
            else if (current_fsm.time_now < 10) current_target_apogee << 5, 0, 0;
            else if (current_fsm.time_now < 19) current_target_apogee << -10, 30, 20;
            else{
                std_msgs::String coast_command;
                coast_command.data = "Coast";
                coast_pub.publish(coast_command);
            }
        } else if (current_fsm.state_machine.compare("Coast") == 0) {
        }

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}