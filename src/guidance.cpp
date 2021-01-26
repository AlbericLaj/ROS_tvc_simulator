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

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/box_admm.hpp"
#include "solvers/admm.hpp"

#include "utils/helpers.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <chrono>

void MPC_guidance();

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
Eigen::Matrix<double, 14, 1> init_cond; // Used for MPC init

// Global variable with array of waypoints = trajectory
tvc_simulator::Waypoint trajectory[N_POINT];







// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

using namespace std;

typedef std::chrono::time_point<std::chrono::system_clock> time_point;
time_point get_time()
{
    /** OS dependent */
#ifdef __APPLE__
    return std::chrono::system_clock::now();
#else
    return std::chrono::high_resolution_clock::now();
#endif
}

#define POLY_ORDER 5
#define NUM_SEG    2
#define NUM_EXP    1

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ guidance_ocp, /*NX*/ 7, /*NU*/ 3, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double) // Was with 4 states, 2 inputs before

using namespace Eigen;

class guidance_ocp : public ContinuousOCP<guidance_ocp, Approximation, SPARSE>
{
public:
    ~guidance_ocp() = default;
    guidance_ocp()
    {
        Q.setZero();
        R.setZero();
        Q.diagonal() << 1.0, 1.0, 15, 0.2, 0.2, 0.5, 2.0;
        R.diagonal() << 0.1, 0.1, 0.3;
        P.setIdentity();
        P.diagonal() << 1.0, 1.0, 100, 0.2, 0.2, 0.5, 5.0;

        xs << 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0;
        us << 0.0, 0.0, 40*9.81;
    }

    static constexpr double t_start = 0;
    static constexpr double t_stop  = 30.0;

    Eigen::Matrix<scalar_t, 7,7> Q;
    Eigen::Matrix<scalar_t, 3,3> R;
    Eigen::Matrix<scalar_t, 7,7> P;

    Eigen::Matrix<scalar_t, 7,1> xs;
    Eigen::Matrix<scalar_t, 3,1> us;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        // -------------- Constant Rocket parameters ----------------------------------------
        //Eigen::Matrix<double, 7, 1> init_cond; init_cond << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1;

        T dry_mass = (T)40;                         // final rocket mass in [kg]
        T propeller_mass = (T)3.1;                  // propellant mass in [kg]

        T Isp = (T)211;                             // Specific Impulse in [s]

        Eigen::Array<T, 3, 1> Cd; Cd << (T)1.5, (T)2.5, (T)0.7;

        T diameter = (T)15.6e-2;                    // Rocket diameter in [m]
        T length = (T)4;                            // Rocket length in [m]
        T surface_front = (T)(3.14159*(diameter/2)*(diameter/2)); // Cross-section surface in [m^2]
        T surface_side = (T)(length*diameter);      // Surface of the rocket seen from the side [m^2]

        Eigen::Array<T, 3, 1> surface; surface << surface_side, surface_side, surface_front;

        // -------------- Constant external parameters ------------------------------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]
        //T rho_air = (T)(353*pow(1-0.0000225577*x(2), x(1))/(288.15-0.0065*x(2)));
        T rho_air = (T)1.225;
        T TIMEUNITS_PER_HOUR = (T)3600.0;           // Conversion hours to seconds

        // -------------- Simulation variables -----------------------------
        T mass = dry_mass + x(6);                   // Instantaneous mass of the rocekt in [kg]
        Eigen::Array<T, 3, 1> speed; speed << (T)x(3), (T)x(4), (T)x(5);

        // External force in 3D in [N]

        Eigen::Matrix<T, 3, 1> drag;
        drag = (Eigen::Matrix<T, 3, 1>)((T)0.5*Cd*surface*rho_air*speed*speed);

        //std::cout << "Drag: " <<  drag.transpose() << "\n";
        //std::cout << "Speed: " <<  speed.transpose() << "\n\n";

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T)0, (T)0, g0*mass;

        // Total force in 3D in [N]
        Eigen::Matrix<T, 3, 1> total_force;
        total_force = u - drag - gravity;

        // -------------- Differential equation ---------------------

        // Position variation is mass
        xdot(0) = x(3);
        xdot(1) = x(4);
        xdot(2) = x(5);

        // Speed variation is Force/mass
        xdot(3) = total_force(0)/mass;
        xdot(4) = total_force(1)/mass;
        xdot(5) = total_force(2)/mass;

        // Mass variation is proportional to thrust
        xdot(6) = -u(2)/(Isp*g0);
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                   const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept
    {
        lagrange = (x - xs.template cast<T>()).dot(Q.template cast<T>() * (x - xs.template cast<T>())) +
                   (u - us.template cast<T>()).dot(R.template cast<T>() * (u - us.template cast<T>()));
    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept
    {
        mayer = (x - xs.template cast<T>()).dot(P.template cast<T>() * (x - xs.template cast<T>()));
    }
};





// --------------------------- Create solver -----------------------------------------------
template<typename Problem, typename QPSolver> class MySolver;

template<typename Problem, typename QPSolver = boxADMM<Problem::VAR_SIZE, Problem::DUAL_SIZE, typename Problem::scalar_t>>
class MySolver : public SQPBase<MySolver<Problem, QPSolver>, Problem, QPSolver>
{
public:
    using Base = SQPBase<MySolver<Problem, QPSolver>, Problem, QPSolver>;
    using typename Base::scalar_t;
    using typename Base::nlp_variable_t;
    using typename Base::nlp_hessian_t;

    /** change step size selection algorithm */
    scalar_t step_size_selection_impl(const Ref<const nlp_variable_t>& p) noexcept
    {
        //std::cout << "taking NEW implementation \n";
        scalar_t mu, phi_l1, Dp_phi_l1;
        nlp_variable_t cost_gradient = this->m_h;
        const scalar_t tau = this->m_settings.tau; // line search step decrease, 0 < tau < settings.tau

        scalar_t constr_l1 = this->constraints_violation(this->m_x);

        // TODO: get mu from merit function model using hessian of Lagrangian
        //const scalar_t quad_term = p.dot(this->m_H * p);
        //const scalar_t qt = quad_term >= 0 ? scalar_t(0.5) * quad_term : 0;
        //mu = (abs(cost_gradient.dot(p)) ) / ((1 - this->m_settings.rho) * constr_l1);

        mu = this->m_lam_k.template lpNorm<Eigen::Infinity>();

        //std::cout << "mu: " << mu << "\n";

        scalar_t cost_1;
        this->problem.cost(this->m_x, this->m_p, cost_1);

        //std::cout << "l1: " << constr_l1 << " cost: " << cost_1 << "\n";

        phi_l1 = cost_1 + mu * constr_l1;
        Dp_phi_l1 = cost_gradient.dot(p) - mu * constr_l1;

        scalar_t alpha = scalar_t(1.0);
        scalar_t cost_step;
        nlp_variable_t x_step;
        for (int i = 1; i < this->m_settings.line_search_max_iter; i++)
        {
            x_step.noalias() = alpha * p;
            x_step += this->m_x;
            this->problem.cost(x_step, this->m_p, cost_step);

            //std::cout << "i: " << i << " l1: " << this->constraints_violation(x_step) << " cost: " << cost_step << "\n";

            scalar_t phi_l1_step = cost_step + mu * this->constraints_violation(x_step);

            //std::cout << "phi before: " << phi_l1 << " after: " << phi_l1_step <<  " required diff: " << alpha * this->m_settings.eta * Dp_phi_l1 << "\n";

            if (phi_l1_step <= (phi_l1 + alpha * this->m_settings.eta * Dp_phi_l1))
            {
                // accept step
                return alpha;
            } else {
                alpha = tau * alpha;
            }
        }

        return alpha;
    }

    /** change Hessian update algorithm to the one provided by ContinuousOCP*/
    EIGEN_STRONG_INLINE void hessian_update_impl(Eigen::Ref<nlp_hessian_t> hessian, const Eigen::Ref<const nlp_variable_t>& x_step,
                                                 const Eigen::Ref<const nlp_variable_t>& grad_step) noexcept
    {
        this->problem.hessian_update_impl(hessian, x_step, grad_step);
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------













// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;

	init_cond << rocket_state->pose.position.x, rocket_state->pose.position.y, rocket_state->pose.position.z,
								rocket_state->twist.linear.x, rocket_state->twist.linear.y, rocket_state->twist.linear.z,
								rocket_state->pose.orientation.w, rocket_state->pose.orientation.x, rocket_state->pose.orientation.y, rocket_state->pose.orientation.z, 
								rocket_state->twist.angular.x, rocket_state->twist.angular.y, rocket_state->twist.angular.z,
								rocket_state->propeller_mass;
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


    res.target_point.speed.x = trajectory[prev_point].speed.x +  ratio* (trajectory[prev_point+1].speed.x - trajectory[prev_point].speed.x);

    res.target_point.speed.y = trajectory[prev_point].speed.y +  ratio* (trajectory[prev_point+1].speed.y - trajectory[prev_point].speed.y);

    res.target_point.speed.z = trajectory[prev_point].speed.z +  ratio* (trajectory[prev_point+1].speed.z - trajectory[prev_point].speed.z);

    res.target_point.propeller_mass = trajectory[prev_point].propeller_mass +  ratio* (trajectory[prev_point+1].propeller_mass - trajectory[prev_point].propeller_mass);

  }
  // If asked for a time before first point, give first point
  else if (prev_point <0)
  {
    res.target_point = trajectory[1];
  }
  // If asked for a time after first point, give last point
  else
  {
    res.target_point = trajectory[N_POINT -1];
  }

  res.target_point.time = req.target_time;

	//std::cout << res.target_point << "\n--------------------------\n";
	
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
  ros::Timer guidance_thread = n.createTimer(ros::Duration(0.5),
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
        //affine_guidance(rocket);
				MPC_guidance();
			}

      else if (current_fsm.state_machine.compare("Coast") == 0)
		  {
        //affine_guidance(rocket);
				MPC_guidance();
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

void MPC_guidance()
{
  using admm = boxADMM<guidance_ocp::VAR_SIZE, guidance_ocp::NUM_EQ, guidance_ocp::scalar_t,
                 guidance_ocp::MATRIXFMT, linear_solver_traits<guidance_ocp::MATRIXFMT>::default_solver>;

	MySolver<guidance_ocp, admm> solver;
	solver.settings().max_iter = 10;
	solver.settings().line_search_max_iter = 10;

	Eigen::Matrix<double, 3, 1> lbu, ubu;
	lbu << -200.0, -200.0, 0.0;
	ubu << 200.0, 200.0, 2000.0;

	solver.upper_bound_x().segment(70, 7) = init_cond;
	solver.lower_bound_x().segment(70, 7) = init_cond;

	solver.primal_solution().head<77>() = init_cond.replicate(11, 1);

	solver.upper_bound_x().tail(33) = ubu.replicate(11,1);
	solver.lower_bound_x().tail(33) = lbu.replicate(11,1);

	// Solve problem and save solution
	double time_now = ros::Time::now().toSec();
	solver.solve();
	//ROS_INFO("Solver duration: %f\n",  ros::Time::now().toSec()-time_now);

	Eigen::Matrix<double, 7, 11> trajectory_MPC; trajectory_MPC << Eigen::Map<Eigen::Matrix<double, 7, 11>>(solver.primal_solution().head(77).data(), 7, 11);

  // Fill the trajectory points' position
  int i = 0;
  for(i = 0; i<N_POINT; i++)
  {
    trajectory[i].time = 0.5*(2*current_fsm.time_now + 30) - 0.5*(30)*cos(3.14159*(2*i+1)/(2*N_POINT));

    trajectory[i].position.x = trajectory_MPC(0, trajectory_MPC.cols()-1 - i);
    trajectory[i].position.y = trajectory_MPC(1, trajectory_MPC.cols()-1 - i);
    trajectory[i].position.z = trajectory_MPC(2, trajectory_MPC.cols()-1 - i);

    trajectory[i].speed.x = trajectory_MPC(3, trajectory_MPC.cols()-1 - i);
    trajectory[i].speed.y = trajectory_MPC(4, trajectory_MPC.cols()-1 - i);
    trajectory[i].speed.z = trajectory_MPC(5, trajectory_MPC.cols()-1 - i);

    trajectory[i].propeller_mass = trajectory_MPC(6, trajectory_MPC.cols()-1 - i);

  }

	//std::cout << trajectory[3] << "\n";

}
