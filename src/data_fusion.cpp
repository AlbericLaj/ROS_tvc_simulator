#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Control.h"
#include "tvc_simulator/Sensor.h"

#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <sstream>
#include <string>


// Global variable with last received fsm
tvc_simulator::FSM current_fsm;

// Global variable with last received control
tvc_simulator::Control current_control;

// Global variable with last received sensor data
tvc_simulator::Sensor current_sensor;

// Callback function to store last received fsm
void fsmCallback(const tvc_simulator::FSM::ConstPtr& fsm)
{
	current_fsm.time_now = fsm->time_now;
  current_fsm.state_machine = fsm->state_machine;
}

// Callback function to store last received control
void controlCallback(const tvc_simulator::Control::ConstPtr& control)
{
	current_control.torque = control->torque;
	current_control.force = control->force;
}


// Callback function to store last received sensor data
void sensorCallback(const tvc_simulator::Sensor::ConstPtr& sensor)
{
	current_sensor.IMU_acc = sensor->IMU_acc;
	current_sensor.IMU_gyro = sensor->IMU_gyro;
	current_sensor.baro_height = sensor->baro_height;
}

int main(int argc, char **argv)
{
	// Init ROS time keeper node
  ros::init(argc, argv, "data_fusion");
  ros::NodeHandle n;

	// Create filtered rocket state publisher
	ros::Publisher kalman_pub = n.advertise<tvc_simulator::State>("kalman_pub", 10);

	// Subscribe to time_keeper for fsm and time
  ros::Subscriber fsm_sub = n.subscribe("fsm_pub", 100, fsmCallback);

	// Subscribe to control for kalman estimator
  ros::Subscriber control_sub = n.subscribe("control_pub", 100, controlCallback);

	// Subscribe to sensor for kalman correction
  ros::Subscriber sensor_sub = n.subscribe("sensor_pub", 100, sensorCallback);

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

  // Thread to compute kalman. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.060),
  [&](const ros::TimerEvent&) 
	{
		ROS_INFO("State: %s", current_fsm.state_machine.c_str());
		// Rocket state X and input U:
		static double X[14];
		double U[4] = {current_control.force.x, current_control.force.y, current_control.force.z, current_control.torque.z};

		// Sensor data
		double IMU_acc[3] = {current_sensor.IMU_acc.x, current_sensor.IMU_acc.y, current_sensor.IMU_acc.z};
		double IMU_gyro[3] = {current_sensor.IMU_gyro.x, current_sensor.IMU_gyro.y, current_sensor.IMU_gyro.z};
		double baro_height = current_sensor.baro_height;

    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else
		{


			// Do Kalman stuff
			X[2] = baro_height;


		}
		// Parse kalman state and publish it on the /kalman_pub topic
		tvc_simulator::State kalman_state;

		kalman_state.pose.position.x = X[0];
		kalman_state.pose.position.y = X[1];
		kalman_state.pose.position.z = X[2];

		kalman_state.twist.linear.x = X[3];
		kalman_state.twist.linear.y = X[4];
		kalman_state.twist.linear.z = X[5];

		kalman_state.pose.orientation.x = X[6];
		kalman_state.pose.orientation.y = X[7];
		kalman_state.pose.orientation.z = X[8];
		kalman_state.pose.orientation.w = X[9];

		kalman_state.twist.angular.x = X[10];
		kalman_state.twist.angular.y = X[11];
		kalman_state.twist.angular.z = X[12];

		kalman_state.propeller_mass = X[13];

	
		kalman_pub.publish(kalman_state);

  });

	// Automatic callback of service and publisher from here
	ros::spin();

}
