#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"

#include "tvc_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "tvc_simulator/GetFSM.h"

#include <time.h>
#include <sstream>
#include <string>

#define CONTROL_HORIZON 5 // In seconds

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/box_admm.hpp"
#include "solvers/admm.hpp"
#include "solvers/osqp_interface.hpp"
#include "control/mpc_wrapper.hpp"

#include "utils/helpers.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace Eigen;


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

  //Rocket();

    void init(ros::NodeHandle n)
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

Rocket rocket;

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


Eigen::Matrix<double, 4,1> convertControl_SI(const Eigen::Matrix<double, 4,1>& u)
{
  Eigen::Matrix<double, 4,1> input; input << 5*u(0), 5*u(1), 15*(u(2)+1), 5*u(3);

  return input;
}








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

POLYMPC_FORWARD_DECLARATION(/*Name*/ control_ocp, /*NX*/ 9, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)


class control_ocp : public ContinuousOCP<control_ocp, Approximation, SPARSE>
{
public:
    ~control_ocp() = default;
    control_ocp()
    {
        Q.setZero();
        R.setZero();
        Q.diagonal() << 50,  0,    500, 500, 500, 500,     100, 100, 100;
        R.diagonal() << 0, 0, 0, 0;
        P.setIdentity();
        P.diagonal() << 50,  0,    500, 500, 500, 500,     100, 100, 100;

        xs << 5/1000,
              0,
              0, 0, 0, 1,
              0, 0, 0;

        us << 0.0, 0.0, 1.0, 0.0;
    }

    static constexpr double t_start = 0.0;
    static constexpr double t_stop  = CONTROL_HORIZON;

    Eigen::Matrix<scalar_t, 9,9> Q;
    Eigen::Matrix<scalar_t, 4,4> R;
    Eigen::Matrix<scalar_t, 9,9> P;

    Eigen::Matrix<scalar_t, 9,1> xs;
    Eigen::Matrix<scalar_t, 4,1> us;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        //std::cout << x.transpose() << "\n" << u.transpose() << "\n\n";
        Eigen::Matrix<T, 4,1> input; input << 5*u(0), 5*u(1),15*(u(2)+1) , 5*u(3);
        
        // -------------- Constant Rocket parameters ----------------------------------------
        Eigen::Matrix<T, 3, 1> J_inv; J_inv << (T)(0.4/0.125), (T)(0.4/0.125), (T)(1.0/0.02);

        // -------------- Constant external parameters ------------------------------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]


        // -------------- Simulation variables -----------------------------
        T mass = (T)2;                  // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude( x(5), x(2), x(3), x(4));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Z drag --> Big approximation: speed in Z is basically the same between world and rocket
        //T drag = (T)(1e6*0.5*0.3*0.0186*1.225*x(1)*x(1)); 
        
        // Force in body frame (drag + thrust) in [N]
        Eigen::Matrix<T, 3, 1> rocket_force; rocket_force << (T)input(0), (T)input(1), (T)input(2);

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity; gravity << (T)0, (T)0, g0*mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;  total_force = rot_matrix*rocket_force - gravity;
        //std::cout << "force " << total_force.transpose() << "\n";


        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T)0.0, x(6), x(7), x(8));
        
        // X, Y force and Z torque in body frame   
        Eigen::Matrix<T, 3, 1> rocket_torque; rocket_torque << input(0), input(1), input(3);
        
        
        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot(0) = x(1);

        // Speed variation is Force/mass
        xdot(1) = (T)1e-3*total_force(2)/mass;  

        // Quaternion variation is 0.5*w◦q
        xdot.segment(2, 4) =  (T)0.5*(omega_quat*attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(6, 3) = rot_matrix*(rocket_torque.cwiseProduct(J_inv));
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


// ------ Create solver -------
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




// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------




















int main(int argc, char **argv)
{
  using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
                 control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
                 
  using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;


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
	
  // Initialize control
	tvc_simulator::Control control_law;
	geometry_msgs::Vector3 thrust_force;
	geometry_msgs::Vector3 thrust_torque;

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";
	
  // Initialize rocket class with useful parameters
  rocket.init(n);
  
	// Init MPC ----------------------------------------------------------------------------------------------------------------------
	// Creates solver
  using mpc_t = MPC<control_ocp, MySolver, osqp_solver_t>;
	mpc_t mpc;
	
  mpc.settings().max_iter = 1;
  mpc.settings().line_search_max_iter = 10;
  //mpc.m_solver.settings().max_iter = 1000;
  //mpc.m_solver.settings().scaling = 10;


  // Input constraints
  const double inf = std::numeric_limits<double>::infinity();
  mpc_t::control_t lbu; 
  mpc_t::control_t ubu; 
  
  lbu << -1, -1, -1, -1; // lower bound on control
  ubu << 1, 1, 1, 1; // upper bound on control
  
  //lbu << -inf, -inf, -inf, -inf;
  //ubu <<  inf,  inf,  inf,  inf;
  mpc.control_bounds(lbu, ubu);

  // State constraints
  const double eps = 1e-1;
  mpc_t::state_t lbx; 
  mpc_t::state_t ubx; 

  lbx << -inf,  -inf,   -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf;
  ubx << inf, inf,       0.183+eps,  0.183+eps,  0.183+eps,  1+eps,    inf,  inf,  inf;
  
  lbx << -inf,   -inf,   -inf, -inf, -inf, -inf,   -inf, -inf, -inf;
  ubx << inf,     inf,    inf,  inf,  inf,  inf,    inf,  inf,  inf;
  mpc.state_bounds(lbx, ubx);
  

  // Initial state
  mpc_t::state_t x0;
  x0 << 0,
				0,
				0, 0, 0, 1, 
        0, 0, 0;
  mpc.x_guess(x0.replicate(9,1));		  
  // --------------------------------------------------------------------------------------------------------------------

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.03),
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
      x0 << current_state.pose.position.z/1000,
						current_state.twist.linear.z/1000,
						current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w, 
						current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z; 
      mpc.initial_conditions(x0);

		  // Solve problem and save solution
			double time_now = ros::Time::now().toSec();
		  mpc.solve();
			ROS_INFO("T= %.2f ms, st: %d, iter: %d",  1000*(ros::Time::now().toSec()-time_now), mpc.info().status.value,  mpc.info().iter);
		 

		  // Get state and control solution
		  //Eigen::Matrix<double, 9, 11> trajectory; trajectory << Eigen::Map<Eigen::Matrix<double, 9, 11>>(solver.primal_solution().head(99).data(), 9, 11);
		  //Eigen::Matrix<double, 4, 11> control_MPC_full; control_MPC_full << Eigen::Map<Eigen::Matrix<double, 4, 11>>(solver.primal_solution().tail(44).data(), 4, 11);

			//std::cout << control_MPC_full << "\n";

			Eigen::Matrix<double, 4, 1> control_MPC;
      control_MPC =  mpc.solution_u_at(0).transpose();
			//ROS_INFO("Fx: %f, Fy: %f, Fz: %f, Mx: %f \n",  control_MPC[0], control_MPC[1], control_MPC[2], control_MPC[3]);
			//Eigen::Matrix<double, 4,1> input; input << 100*control_MPC(0), 100*control_MPC(1), 1000*(control_MPC(2)+1), 50*control_MPC(3);
			
			Eigen::Matrix<double, 4,1> input = convertControl_SI(control_MPC);

			// -------------------------------------------------------------------------------------------------------------------------


      // Simple P controller. Thrust in z is used to reach apogee, Thrust in x and y is used to keep vertical orientation
			thrust_force.z = (1000 - current_state.pose.position.z)*100;

      thrust_force.x = current_state.pose.orientation.y*1;
      thrust_force.y = current_state.pose.orientation.x*1;

			thrust_force.x = input[0];
			thrust_force.y = input[1];
			thrust_force.z = input[2];


      if(thrust_force.z > rocket.maxThrust) thrust_force.z = rocket.maxThrust;
      if(thrust_force.z < rocket.minThrust) thrust_force.z = rocket.minThrust;
      
      float max_side_force = 10;
      if(thrust_force.x > max_side_force) thrust_force.x = max_side_force;
      if(thrust_force.x < -max_side_force) thrust_force.x = -max_side_force;
      if(thrust_force.y > max_side_force) thrust_force.y = max_side_force;
      if(thrust_force.y < -max_side_force) thrust_force.y = -max_side_force;

      // Torque in X and Y is defined by thrust in X and Y. Torque in Z is free variable
      thrust_torque.x = thrust_force.x*0.4; 
      thrust_torque.y = thrust_force.y*0.4;
			thrust_torque.z = input[3];

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
		//ROS_INFO("Z force is %f", thrust_force.z);

  });

	// Automatic callback of service and publisher from here
	ros::spin();

}
