#include "ros/ros.h"
#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include <time.h>

#include <sstream>
#include <string>

#include "tvc_simulator/GetFSM.h"
#include "std_msgs/String.h"

// global variable with time and state machine
tvc_simulator::FSM current_fsm;
double time_zero;

// global variable with last received rocket state
tvc_simulator::State current_rocket_state;

void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_rocket_state.pose = rocket_state->pose;
  current_rocket_state.twist = rocket_state->twist;
  current_rocket_state.propeller_mass = rocket_state->propeller_mass;
}

// Service function: send back fsm (time + state machine)
bool sendFSM(tvc_simulator::GetFSM::Request &req, tvc_simulator::GetFSM::Response &res)
{
	// Update current time
	current_fsm.time_now = ros::Time::now().toSec() - time_zero;

	res.fsm.time_now = current_fsm.time_now;
	res.fsm.state_machine = current_fsm.state_machine;

	ROS_INFO("Request info: State = %s, time = %f", current_fsm.state_machine.c_str(), current_fsm.time_now);
	
	return true;
}

void processCommand(const std_msgs::String &command){
    current_fsm.state_machine = "Launch";
    time_zero = ros::Time::now().toSec();
    if(command.data.compare("Launch") == 0){
        //TODO ?
    }
}

int main(int argc, char **argv)
{

	// Init ROS time keeper node
  ros::init(argc, argv, "time_keeper");
  ros::NodeHandle n;


    // Subscribe to commands
    ros::Subscriber command_sub = n.subscribe("commands", 10, processCommand);

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";


	// Create timer service
	ros::ServiceServer timer_service = n.advertiseService("getFSM", sendFSM);

	// Create timer publisher and associated thread (100Hz)
	ros::Publisher timer_pub = n.advertise<tvc_simulator::FSM>("fsm_pub", 10);

	// Subscribe to state message
  ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);

  ros::Timer FSM_thread = n.createTimer(ros::Duration(0.01),
  [&](const ros::TimerEvent&) 
	{
		// Update current time
		current_fsm.time_now = ros::Time::now().toSec() - time_zero;

    // Update FSM
  	if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else if (current_fsm.state_machine.compare("Launch") == 0)
		{
      // End of burn -> no more thrust
      if(current_rocket_state.propeller_mass <0)
      {
        current_fsm.state_machine = "Coast";
      }

    }
    
    else if (current_fsm.state_machine.compare("Coast") == 0)
		{
      // Do nothing for now
    }

    // Publish time + state machine    
		timer_pub.publish(current_fsm);
//	  ROS_INFO("Sent info: State = %s, time = %f", current_fsm.state_machine.c_str(), current_fsm.time_now);

  });

	// Automatic callback of service and publisher from here
	ros::spin();

}
