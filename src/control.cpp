#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Waypoint.h"

#include "tvc_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "tvc_simulator/GetFSM.h"
#include "tvc_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#define CONTROL_HORIZON 1 // In seconds

class Rocket
{
  public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    float maxThrust;
    float minThrust;
    float dry_CM;

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};

  Rocket(ros::NodeHandle n)
  {
    n.getParam("/rocket/maxThrust", maxThrust);
    n.getParam("/rocket/minThrust", minThrust);
    n.getParam("/rocket/Isp", Isp);

    n.getParam("/rocket/dry_mass", dry_mass);
    n.getParam("/rocket/propellant_mass", propellant_mass);
    
    n.getParam("/rocket/Cd", Cd);
    n.getParam("/rocket/dry_CM", dry_CM);

    n.getParam("/environment/apogee", target_apogee);

    std::vector<float> diameter = {0, 0, 0};
    std::vector<float> length = {0, 0, 0};
    int nStage;

    n.getParam("/rocket/diameters", diameter);
    n.getParam("/rocket/stage_z", length);
    n.getParam("/rocket/stages", nStage);

    surface[0] = diameter[1]*length[nStage-1];
    surface[1] = surface[0];
    surface[2] = diameter[1]*diameter[1]/4 * 3.14159;
  }

};

// Global variable with next waypoint to be followed
tvc_simulator::Waypoint target_point;

// Global variable with last received rocket state
tvc_simulator::State current_state;

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}


int main(int argc, char **argv)
{
	// Init ROS time keeper node
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

	// Create control publisher
	ros::Publisher control_pub = n.advertise<tvc_simulator::Control>("control_pub", 10);

	// Subscribe to state message from simulation
  ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);

	// Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
  tvc_simulator::GetFSM srv_fsm;

  // Setup Waypoint client and srv variable for trajectory following
  ros::ServiceClient client_waypoint = n.serviceClient<tvc_simulator::GetWaypoint>("getWaypoint");
  tvc_simulator::GetWaypoint srv_waypoint;
	
  // Initialize control
	tvc_simulator::Control control_law;
	geometry_msgs::Vector3 thrust_force;
	geometry_msgs::Vector3 thrust_torque;

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";
	
  // Initialize rocket class with useful parameters
  Rocket rocket(n);

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.05),
  [&](const ros::TimerEvent&) 
	{
    // Get current FSM and time
    if(client_fsm.call(srv_fsm))
    {
      current_fsm = srv_fsm.response.fsm;
    }
    // Get next waypoint objective
    srv_waypoint.request.target_time = current_fsm.time_now + CONTROL_HORIZON;
    if(client_waypoint.call(srv_waypoint))
    {
      target_point = srv_waypoint.response.target_point;
    }

    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else if (current_fsm.state_machine.compare("Launch") == 0)
		{
      // Simple P controller. Thrust in z is used to reach apogee, Thrust in x and y is used to keep vertical orientation
			thrust_force.z = (target_point.position.z - current_state.pose.position.z)*100;

      thrust_force.x = -current_state.pose.orientation.x*700/rocket.dry_CM;
      thrust_force.y = -current_state.pose.orientation.y*700/rocket.dry_CM;

      if(thrust_force.z > rocket.maxThrust) thrust_force.z = rocket.maxThrust;
      if(thrust_force.z < rocket.minThrust) thrust_force.z = rocket.minThrust;

      // Torque in X and Y is defined by thrust in X and Y. Torque in Z is free variable
      thrust_torque.x = thrust_force.x*rocket.dry_CM;
      thrust_torque.y = thrust_force.y*rocket.dry_CM;

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;
		}

		else if (current_fsm.state_machine.compare("Coast") == 0)
		{
      thrust_force.x = 0;
      thrust_force.y = 0;
      thrust_force.z = 0;

      thrust_torque.x = 0;
      thrust_torque.y = 0;
      thrust_torque.z = 0;

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;
    }
	
		control_pub.publish(control_law);
		ROS_INFO("Z force is %f", thrust_force.z);

  });

	// Automatic callback of service and publisher from here
	ros::spin();

}
