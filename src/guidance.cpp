#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Waypoint.h"

#include "tvc_simulator/GetFSM.h"
#include "tvc_simulator/GetWaypoint.h"

#include <time.h>

#include <sstream>
#include <string>
#include <vector>


#define N_POINT 10 // Number of collocation points

class Rocket
{
  public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    float maxThrust;
    float minThrust;

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

// Simple affine function as trajectory
void affine_guidance(Rocket rocket);

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Global variable with last received rocket state
tvc_simulator::State current_state;

// Global variable with array of waypoints = trajectory
tvc_simulator::Waypoint trajectory[N_POINT];

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}

// Service function: send back waypoint at requested time
// Current version is a very crude linear interpolation from the Npoints stored as trajectory
bool sendWaypoint(tvc_simulator::GetWaypoint::Request &req, tvc_simulator::GetWaypoint::Response &res)
{
  // Find closest point in time compared to requested time
  int i = 0;
  for(i = 0; i<N_POINT; i++)
  {
    if(trajectory[i].time > req.target_time)
    {
      break;
    }
  }
  int prev_point = i-1;
  
  // If requested time lies inside array of points:
  if (prev_point >= 0 && prev_point < N_POINT -1)
  {  
    // How close requested time is to previous or next point. Is between 0 and 1
    float ratio = ((req.target_time - trajectory[prev_point].time)/(trajectory[prev_point+1].time - trajectory[prev_point].time));

    res.target_point.position.x = trajectory[prev_point].position.x +  ratio* (trajectory[prev_point+1].position.x - trajectory[prev_point].position.x);

    res.target_point.position.y = trajectory[prev_point].position.y +  ratio* (trajectory[prev_point+1].position.y - trajectory[prev_point].position.y);

    res.target_point.position.z = trajectory[prev_point].position.z +  ratio* (trajectory[prev_point+1].position.z - trajectory[prev_point].position.z);

  }
  // If asked for a time before first point, give first point
  else if (prev_point <0)
  {
    res.target_point.position.x = trajectory[0].position.x;
    res.target_point.position.y = trajectory[0].position.y;
    res.target_point.position.z = trajectory[0].position.z;
  }
  // If asked for a time after first point, give last point
  else
  {
    res.target_point.position.x = trajectory[N_POINT -1].position.x;
    res.target_point.position.y = trajectory[N_POINT -1].position.y;
    res.target_point.position.z = trajectory[N_POINT -1].position.z;
  }

  res.target_point.time = req.target_time;
	
	return true;
}

int main(int argc, char **argv)
{
	// Init ROS guidance node
  ros::init(argc, argv, "guidance");
  ros::NodeHandle n;


	// Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
  tvc_simulator::GetFSM srv_fsm;

	// Create waypoint service
	ros::ServiceServer waypoint_service = n.advertiseService("getWaypoint", sendWaypoint);

	// Subscribe to state message from simulation
  ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);
	

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

  // Initialize rocket class with useful parameters
  Rocket rocket(n);
	
  // Thread to compute guidance. Duration defines interval time in seconds
  ros::Timer guidance_thread = n.createTimer(ros::Duration(0.1),
  [&](const ros::TimerEvent&) 
	{
      // Get current FSM and time
      if(client_fsm.call(srv_fsm))
      {
        current_fsm = srv_fsm.response.fsm;
      }

      // State machine ------------------------------------------
			if (current_fsm.state_machine.compare("Idle") == 0)
			{
				// Do nothing
			}

			else if (current_fsm.state_machine.compare("Launch") == 0)
			{
        affine_guidance(rocket);
			}

      else if (current_fsm.state_machine.compare("Coast") == 0)
		  {
        affine_guidance(rocket);
      }
      // ---------------------------------------------------------
		
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}

// Creates very basic trajectory to reach desired points at apogee using affine functions
void affine_guidance(Rocket rocket)
{
  // Define affine parameters for position trajectory
  float tf = 30;
  float dT = tf - current_fsm.time_now;
  float a_x = (rocket.target_apogee[0] - current_state.pose.position.x)/dT;
  float a_y = (rocket.target_apogee[1] - current_state.pose.position.y)/dT;
  float a_z = (rocket.target_apogee[2] - current_state.pose.position.z)/dT;

  float b_x = rocket.target_apogee[0] - a_x*tf;
  float b_y = rocket.target_apogee[1] - a_y*tf;
  float b_z = rocket.target_apogee[2] - a_z*tf;

  // Fill the trajectory points' position
  int i = 0;
  for(i = 0; i<N_POINT; i++)
  {
    trajectory[i].time = current_fsm.time_now + i*dT/(N_POINT-1);

    trajectory[i].position.x = a_x*trajectory[i].time + b_x;
    trajectory[i].position.y = a_y*trajectory[i].time + b_y;
    trajectory[i].position.z = a_z*trajectory[i].time + b_z;

  }
}
