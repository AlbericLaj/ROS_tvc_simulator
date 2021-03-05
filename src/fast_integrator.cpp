#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Sensor.h"

#include "tvc_simulator/GetFSM.h"

#include "tvc_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


#include <chrono>
#include <random>

class Rocket
{
  public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    float minTorque;
    float maxTorque;

    std::vector<float> maxThrust{0, 0, 0};
    std::vector<float> minThrust{0, 0, 0};

    float dry_CM;
    float propellant_CM;
    float total_CM; // Current Cm of rocket, in real time

    float total_length;

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};
    std::vector<float> drag_coeff = {0, 0, 0};
    
    std::vector<float> dry_Inertia{0, 0, 0};
    std::vector<float> total_Inertia{0, 0, 0};

    std::vector<float> J_inv{0, 0, 0};

    // Sensor data
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float baro;


    void update_CM( float current_prop_mass)
    {
      total_CM = total_length - (dry_CM*dry_mass + propellant_CM*current_prop_mass)/(dry_mass+current_prop_mass) ; // From aft of rocket

      float new_inertia = dry_Inertia[0] + pow(total_CM-(total_length-propellant_CM), 2)*current_prop_mass;

      total_Inertia[0] = new_inertia;
      total_Inertia[1] = new_inertia;

      J_inv[0] = total_CM/total_Inertia[0];
      J_inv[1] = total_CM/total_Inertia[1];
      J_inv[2] = 1/total_Inertia[2];
    }


    void init(ros::NodeHandle n)
    {
      n.getParam("/rocket/minTorque", minTorque);
      n.getParam("/rocket/maxTorque", maxTorque);
      n.getParam("/rocket/maxThrust", maxThrust);
      n.getParam("/rocket/minThrust", minThrust);
      n.getParam("/rocket/Isp", Isp);

      n.getParam("/rocket/dry_mass", dry_mass);
      n.getParam("/rocket/propellant_mass", propellant_mass);
      
      n.getParam("/rocket/Cd", Cd);
      n.getParam("/rocket/dry_I", dry_Inertia);

      n.getParam("/rocket/dry_CM", dry_CM);
      n.getParam("/rocket/propellant_CM", propellant_CM);

      n.getParam("/environment/apogee", target_apogee);

      std::vector<float> diameter = {0, 0, 0};
      std::vector<float> length = {0, 0, 0};
      
      int nStage;

      n.getParam("/rocket/diameters", diameter);
      n.getParam("/rocket/stage_z", length);
      n.getParam("/rocket/stages", nStage);

      total_length = length[nStage-1];

      surface[0] = diameter[1]*total_length;
      surface[1] = surface[0];
      surface[2] = diameter[1]*diameter[1]/4 * 3.14159;

      float rho_air = 1.225;
      drag_coeff[0] = 0.5*rho_air*surface[0]*Cd[0];
      drag_coeff[1] = 0.5*rho_air*surface[1]*Cd[1];
      drag_coeff[2] = 0.5*rho_air*surface[2]*Cd[2];

      total_Inertia[2] = dry_Inertia[2];

      update_CM(propellant_mass);
    }
};

Rocket rocket;

// Global variable with last updated rocket force & torque
Eigen::Matrix<double, 3,2> rocket_control;

//Global variable with last updated aerodynamic force & torque
Eigen::Matrix<double, 3,2> aero_control;

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Callback function to store last received control
void rocket_controlCallback(const tvc_simulator::Control::ConstPtr& control_law)
{
  rocket_control << control_law->force.x,  control_law->torque.x,
                    control_law->force.y,  control_law->torque.y,
                    control_law->force.z,  control_law->torque.z;
}   

// Callback function to store last received aero force and torque
void rocket_aeroCallback(const tvc_simulator::Control::ConstPtr& rocket_aero)
{
  aero_control <<   rocket_aero->force.x,  rocket_aero->torque.x,
                    rocket_aero->force.y,  rocket_aero->torque.y,
                    rocket_aero->force.z,  rocket_aero->torque.z;
}   

// Callback function to store last received fsm
void fsmCallback(const tvc_simulator::FSM::ConstPtr& fsm)
{
	current_fsm.time_now = fsm->time_now;
  current_fsm.state_machine = fsm->state_machine;
}

void send_fake_sensor(ros::Publisher rocket_sensor_pub)
{
  // construct a trivial random generator engine from a time-based seed:
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::normal_distribution<double> acc_noise (0.0, 0);
  std::normal_distribution<double> gyro_noise (0.0, 0.0);
  std::normal_distribution<double> baro_noise (0.0, 0.0);

  tvc_simulator::Sensor sensor_msg;

  sensor_msg.IMU_acc.x = rocket.accX + acc_noise(generator);
  sensor_msg.IMU_acc.y = rocket.accY + acc_noise(generator);
  sensor_msg.IMU_acc.z = rocket.accZ + acc_noise(generator);

  sensor_msg.IMU_gyro.x = rocket.gyroX + gyro_noise(generator);
  sensor_msg.IMU_gyro.y = rocket.gyroY + gyro_noise(generator);
  sensor_msg.IMU_gyro.z = rocket.gyroZ + gyro_noise(generator);

  sensor_msg.baro_height = rocket.baro + baro_noise(generator);

  rocket_sensor_pub.publish(sensor_msg);
}


using namespace std;
using namespace boost::numeric::odeint;



template<typename scalar_t>
using state_t = Eigen::Matrix<scalar_t, 14, 1>;

using state = state_t<double>;

template<typename scalar_t>
using control_t = Eigen::Matrix<scalar_t, 4, 1>;

void dynamics_flight(const state& x, state& xdot, const double &t)  
{
  // -------------- Simulation variables -----------------------------
  double g0 = 3.986e14/pow(6371e3+x(2), 2);  // Earth gravity in [m/s^2]

  double mass = rocket.dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]

  // Orientation of the rocket with quaternion
  Eigen::Quaternion<double> attitude( x(9), x(6), x(7), x(8)); attitude.normalize();
  Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
 

  // Force in inertial frame: gravity
  Eigen::Matrix<double, 3, 1> gravity; gravity << 0, 0, g0*mass;

  // Total force in inertial frame [N]
  Eigen::Matrix<double, 3, 1> total_force;  total_force = rot_matrix*rocket_control.col(0) - gravity + aero_control.col(0);
  //std::cout << total_force.transpose() << "\n";
  

  // Angular velocity omega in quaternion format to compute quaternion derivative
  Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));
  //std::cout << x.segment(10,3).transpose()*57.29 << "\n\n";

  // Tortal torque in body frame
  Eigen::Matrix<double, 3, 1> I_inv; I_inv << 1/rocket.total_Inertia[0], 1/rocket.total_Inertia[1], 1/rocket.total_Inertia[2]; 

  Eigen::Matrix<double, 3, 1> total_torque; 
  total_torque = rocket_control.col(1) + rot_matrix.transpose()*aero_control.col(1);

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
  xdot(13) = -rocket_control.col(0).norm()/(rocket.Isp*g0);


  // Fake sensor data update -----------------
  Eigen::Matrix<double, 3, 1> body_acceleration;
  body_acceleration = rot_matrix.transpose()*(total_force+gravity)/mass;
  rocket.accX = body_acceleration(0);
  rocket.accY = body_acceleration(1);
  rocket.accZ = body_acceleration(2);

  Eigen::Matrix<double, 3, 1> body_gyroscope;
  body_gyroscope = rot_matrix.transpose()*x.segment(10,3);

  rocket.gyroX = body_gyroscope(0);
  rocket.gyroY = body_gyroscope(1);
  rocket.gyroZ = body_gyroscope(2);

  rocket.baro = x(2);
  }


void dynamics_rail(const state& x, state& xdot, const double &t)  
{
  // -------------- Simulation variables -----------------------------
  double g0 = 3.986e14/pow(6371e3+x(2), 2);  // Earth gravity in [m/s^2]

  double mass = rocket.dry_mass + x(13);     // Instantaneous mass of the rocket in [kg]

  // Orientation of the rocket with quaternion
  Eigen::Quaternion<double> attitude( x(9), x(6), x(7), x(8)); attitude.normalize();
  Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

  // Force in inertial frame: gravity
  Eigen::Matrix<double, 3, 1> gravity; gravity << 0, 0, g0*mass;

  // Total force in initial body frame [N] (rail frame)
  Eigen::Matrix<double, 3, 1> total_force;  total_force = rocket_control.col(0) - rot_matrix.transpose()*(gravity + aero_control.col(0));

  Eigen::Matrix<double, 3, 1> body_acceleration;

  total_force.head(2) << 0.0, 0.0; // Zero force on axes perpendicular to rail to force rocket to stay on rail
  //std::cout << total_force << "\n";


  // Angular velocity omega in quaternion format to compute quaternion derivative
  Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));

  // -------------- Differential equation ---------------------

  // Position variation is speed
  xdot.head(3) = x.segment(3,3);

  // Speed variation is Force/mass
  xdot.segment(3,3) = rot_matrix*total_force/mass;  

  // Quaternion variation is zero to keep rail orientation
  xdot.segment(6, 4) << 0.5*(omega_quat*attitude).coeffs();

  // Angular speed variation is zero to keep rail orientation
  xdot.segment(10, 3) << 0.0, 0.0, 0.0;

  // Mass variation is proportional to total thrust
  xdot(13) = -rocket_control.col(0).norm()/(rocket.Isp*g0);

  // Fake sensor data update -----------------
  body_acceleration = (total_force + rot_matrix.transpose()*gravity)/mass; // Here gravity is measured by the accelerometer because of the rail's reaction force
  rocket.accX = body_acceleration(0);
  rocket.accY = body_acceleration(1);
  rocket.accZ = body_acceleration(2);

  rocket.gyroX = 0.0;
  rocket.gyroY = 0.0;
  rocket.gyroZ = 0.0;

  rocket.baro = x(2);
}


typedef runge_kutta_dopri5< double > stepper_type;
using stepper_type2 = runge_kutta_dopri5<state>; 


int main(int argc, char **argv)
{
  /* ---------- ROS intitialization ---------- */

  // Init ROS fast integrator node
  ros::init(argc, argv, "fast_integrator");
  ros::NodeHandle n;

  // Subscribe to control message from control node
  ros::Subscriber rocket_control_sub = n.subscribe("control_pub", 100, rocket_controlCallback);

  // Subscribe to aero message 
  ros::Subscriber rocket_aero_sub = n.subscribe("rocket_aero", 100, rocket_aeroCallback);

  // Subscribe to time_keeper for fsm and time
  ros::Subscriber fsm_sub = n.subscribe("fsm_pub", 100, fsmCallback);

	// Create fast state publisher
	ros::Publisher rocket_state_pub = n.advertise<tvc_simulator::State>("rocket_state", 10);

  // Create fake sensors publisher
	ros::Publisher rocket_sensor_pub = n.advertise<tvc_simulator::Sensor>("sensor_pub", 10);

  // Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
  tvc_simulator::GetFSM srv_fsm;

  /* ---------- Variable initialization  ---------- */
  
  // Initialize rocket class with useful parameters
  rocket.init(n);

  // Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

  // Initialize external forces
  aero_control << 0, 0,
                  0, 0,
                  0, 0;

  rocket_control << 0, 0,
                    0, 0,
                    0, 0;

  using namespace Eigen;

  //Get initial orientation and convert in Radians
  float roll = 0, zenith = 0, azimuth = 0.0;
  n.getParam("/environment/rocket_roll", roll);
  n.getParam("/environment/rail_zenith", zenith);
  n.getParam("/environment/rail_azimuth", azimuth);

  roll *= 3.14159/180; zenith *= 3.14159/180; azimuth *= 3.14159/180;

  typedef EulerSystem<-EULER_Z, EULER_Y, EULER_Z> Rail_system;
  typedef EulerAngles<double, Rail_system> angle_type;

  angle_type init_angle(azimuth, zenith, roll);

  Quaterniond q(init_angle);

  // Init state X   
  state X0;
  X0 << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,      0, 0, 0.0,    rocket.propellant_mass;
  X0.segment(6,4) = q.coeffs();

  state xout = X0;

  // Init solver
  float period_integration = 10e-3; 
  stepper_type2 stepper;     

  // Thread to integrate state. Duration defines interval time in seconds
  ros::Timer integrator_thread = n.createTimer(ros::Duration(period_integration), [&](const ros::TimerEvent&) 
	{
    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			
		}
    else
    {
      
      if (current_fsm.state_machine.compare("Rail") == 0)
      {
        stepper.do_step(dynamics_rail, X0, 0, xout, 0 + period_integration);
      }

      else if (current_fsm.state_machine.compare("Launch") == 0)
      {
        stepper.do_step(dynamics_flight, X0, 0, xout, 0 + period_integration);
      }

      else if (current_fsm.state_machine.compare("Coast") == 0)
      {
        rocket_control << 0, 0,
                          0, 0,
                          0, 0;
        stepper.do_step(dynamics_flight, X0, 0, xout, 0 + period_integration);
      }

      
      X0 = xout;

      rocket.update_CM(X0(13));

      send_fake_sensor(rocket_sensor_pub);
    }
		

    // Parse state and publish it on the /fast_rocket_state topic
    tvc_simulator::State current_state;

    current_state.pose.position.x = X0(0);
    current_state.pose.position.y = X0(1);
    current_state.pose.position.z = X0(2);

    current_state.twist.linear.x = X0(3);
    current_state.twist.linear.y = X0(4);
    current_state.twist.linear.z = X0(5);

    current_state.pose.orientation.x = X0(6);
    current_state.pose.orientation.y = X0(7);
    current_state.pose.orientation.z = X0(8);
    current_state.pose.orientation.w = X0(9);

    current_state.twist.angular.x = X0(10);
    current_state.twist.angular.y = X0(11);
    current_state.twist.angular.z = X0(12);

    current_state.propeller_mass = X0(13);

    rocket_state_pub.publish(current_state);

    //std::cout << "Fast integration time: " << 1000*(ros::Time::now().toSec()-time_now) << "ms \n";
  });

  // Automatic callback of service and publisher from here
	ros::spin();

}