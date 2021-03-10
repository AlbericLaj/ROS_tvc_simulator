#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Control.h"
#include "tvc_simulator/Sensor.h"

#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iostream>
#include <chrono>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "autodiff/AutoDiffScalar.h"

#include <type_traits>



// Global variable with last received fsm
tvc_simulator::FSM current_fsm;

// Global variable with last received control
tvc_simulator::Control current_control;

// Global variable with last received sensor data
tvc_simulator::Sensor current_sensor;


template<typename scalar_t>
using state_t = Eigen::Matrix<scalar_t, 14, 1>;

using ADScalar = Eigen::AutoDiffScalar<state_t<double>>;
using ad_state_t = state_t<ADScalar>;

template<typename scalar_t>
using sensor_t = Eigen::Matrix<scalar_t, 7, 1>;

using ADScalar_Sensor = Eigen::AutoDiffScalar<sensor_t<double>>;
using ad_state_sensor_t = sensor_t<ADScalar_Sensor>;

using namespace Eigen;
typedef Matrix<double, 14, 14> state_matrix;
typedef Matrix<double, 14, 1> state_vector;


class EKF {

	public:
		state_matrix Q;
		Matrix<double, 7, 7> R;

		state_matrix P;

		ad_state_t X, Xdot;

		state_matrix Jac_f;

		void init_EKF()
		{
			X << 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 1.0,    0.0, 0.0, 0.0,   6;

			/** initialize derivatives */
			int div_size = X.size();
			int derivative_idx = 0;
			for(int i = 0; i < X.size(); ++i)
			{
				X(i).derivatives() =  state_t<double>::Unit(div_size, derivative_idx);
				derivative_idx++;
			}

			P.setZero();

			Q.setIdentity();
			Q.diagonal() << 10, 10, 10,   5, 5, 5,   5, 5, 5, 5,   1, 1, 1,  5; 

			R.setIdentity();
			R.diagonal() << 1.0, 1.0, 1.0,  1.0, 1.0, 1.0,   5.0;
		}
		
		template<typename T>
		void state_dynamics(const state_t<T>& x, state_t<T>& xdot, const double &t)
		{
			/** constants */
			const double dry_mass = 41.558;

			Eigen::Matrix<double, 3, 1> rocket_control;
			rocket_control << current_control.force.x, current_control.force.y, current_control.force.z;
			
			Eigen::Matrix<double, 3, 1> I_inv; I_inv << 1.0/47, 1.0/47, 1.0/2;

			Eigen::Matrix<double, 3, 1> total_torque;
			total_torque << current_control.torque.x, current_control.torque.y, current_control.torque.z;
			total_torque << 0.0, 0.0, 0.0;
			
			const double Isp = 213;

			// -------------- Simulation variables -----------------------------
			T g0 = 3.986e14/pow(6371e3+x(2), 2);  // Earth gravity in [m/s^2]
			T mass = dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

			// Force in inertial frame: gravity
			Eigen::Matrix<T, 3, 1> gravity; gravity << 0, 0, g0*mass;

			// Total force in inertial frame [N]
			Eigen::Matrix<T, 3, 1> total_force;  total_force = rocket_control-gravity;

			// Angular velocity omega in quaternion format to compute quaternion derivative
			Eigen::Quaternion<T> omega_quat(0.0, x(10), x(11), x(12));

			// -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is Force/mass
			xdot.segment(3,3) = total_force/mass;

			// Quaternion variation is 0.5*w◦q
			xdot.segment(6, 4) =  0.5*(omega_quat*attitude).coeffs();

			// Angular speed variation is Torque/Inertia
			xdot.segment(10, 3) = rot_matrix*(total_torque.cwiseProduct(I_inv));

			// Mass variation is proportional to total thrust
			xdot(13) = -rocket_control.norm()/(Isp*g0);
		}

		template<typename T>
		void sensor_estimation(const state_t<T>& x, state_t<T>& z_est, const double &t)
		{
			/** constants */
			const double dry_mass = 41.558;

			// -------------- Simulation variables -----------------------------
			T mass = dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]

			Eigen::Matrix<double, 3, 1> rocket_control;
			rocket_control << current_control.force.x, current_control.force.y, current_control.force.z;

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<T> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();
			
			// Estimate sensor based on current state
			z_est.head(3) = rocket_control/mass;

			z_est.segment(3,3) = rot_matrix.transpose()*x.segment(10,3);

			z_est(6) = x(2);
		}

		void RK4(double dT)
		{
			ad_state_t k1, k2, k3, k4;

			state_dynamics<ADScalar>(X, k1,0);
			state_dynamics<ADScalar>(X+k1*dT/2, k2,0);
			state_dynamics<ADScalar>(X+k2*dT/2, k3,0);
			state_dynamics<ADScalar>(X+k3*dT, k4,0);

			X = X + (k1+2*k2+2*k3+k4)*dT/6;
			 
		}

		void predict_step()
		{
			static double last_predict_time = ros::Time::now().toSec();

			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT);
			last_predict_time = ros::Time::now().toSec();

			// propagate through the function
			state_dynamics<ADScalar>(X, Xdot,0);

			// obtain the jacobian
			for(int i = 0; i < X.size(); i++)
			{
				Jac_f.row(i) = Xdot(i).derivatives();
			}

			P = P + dT*(Jac_f*P + P*Jac_f.transpose() + Q);
		}

		void update_step(Matrix<double, 7, 1> z)
		{
			ad_state_t Z_Est;

			// propagate through the function
			sensor_estimation<ADScalar>(X, Z_Est,0);

			// obtain the jacobian
			Matrix<double, 7, 14> Jac_h;
			for(int i = 0; i < 7; i++)
			{
				Jac_h.row(i) = Z_Est(i).derivatives().head(14);
			}

			Matrix<double, 14, 7> K;
			K = P*Jac_h.transpose()*((Jac_h*P*Jac_h.transpose() + R).inverse());

			X = X + K*(z - Z_Est.head(7));

			std::cout << (z - Z_Est.head(7)).transpose() << "\n\n";
		}


};
EKF kalman;

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

	Matrix<double, 7, 1> new_data;
	new_data << 	sensor->IMU_acc.x, sensor->IMU_acc.y, sensor->IMU_acc.z,  
			sensor->IMU_gyro.x, sensor->IMU_gyro.y, sensor->IMU_gyro.z,  
			sensor->baro_height;

	kalman.update_step(new_data);
}


int main(int argc, char **argv)
{
	// Init ROS time keeper node
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle n;

	// Create filtered rocket state publisher
	ros::Publisher kalman_pub = n.advertise<tvc_simulator::State>("kalman_rocket_state", 10);

	// Subscribe to time_keeper for fsm and time
	ros::Subscriber fsm_sub = n.subscribe("fsm_pub", 100, fsmCallback);

	// Subscribe to control for kalman estimator
	ros::Subscriber control_sub = n.subscribe("control_pub", 100, controlCallback);

	// Subscribe to sensor for kalman correction
	ros::Subscriber sensor_sub = n.subscribe("sensor_pub", 100, sensorCallback);

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	// init EKF
	kalman.init_EKF();

	// Thread to compute kalman. Duration defines interval time in seconds
	ros::Timer control_thread = n.createTimer(ros::Duration(0.100),
	[&](const ros::TimerEvent&) 
	{


	// State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else
		{
			kalman.predict_step();
		}
		// Parse kalman state and publish it on the /kalman_pub topic
		tvc_simulator::State kalman_state;

		kalman_state.pose.position.x = kalman.X(0).value();
		kalman_state.pose.position.y = kalman.X(1).value();
		kalman_state.pose.position.z = kalman.X(2).value();

		kalman_state.twist.linear.x = kalman.X(3).value();
		kalman_state.twist.linear.y = kalman.X(4).value();
		kalman_state.twist.linear.z = kalman.X(5).value();

		kalman_state.pose.orientation.x = kalman.X(6).value();
		kalman_state.pose.orientation.y = kalman.X(7).value();
		kalman_state.pose.orientation.z = kalman.X(8).value();
		kalman_state.pose.orientation.w = kalman.X(9).value();

		kalman_state.twist.angular.x = kalman.X(10).value();
		kalman_state.twist.angular.y = kalman.X(11).value();
		kalman_state.twist.angular.z = kalman.X(12).value();

		kalman_state.propeller_mass = kalman.X(13).value();


		kalman_pub.publish(kalman_state);

	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
