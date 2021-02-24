#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Waypoint.h"
#include "tvc_simulator/Trajectory.h"

#include "tvc_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "tvc_simulator/GetFSM.h"
#include "tvc_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#define CONTROL_HORIZON 3 // In seconds

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/box_admm.hpp"
#include "solvers/admm.hpp"
#include "solvers/osqp_interface.hpp"

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

// Global variable with next waypoint to be followed
tvc_simulator::Waypoint target_point;

// Global variable with last received rocket state
tvc_simulator::State current_state;
Eigen::Matrix<double, 14, 1> init_cond; // Used for MPC init

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;

	init_cond << rocket_state->pose.position.x/1000, rocket_state->pose.position.y/1000, rocket_state->pose.position.z/1000,
								rocket_state->twist.linear.x/1000, rocket_state->twist.linear.y/1000, rocket_state->twist.linear.z/1000,
								rocket_state->pose.orientation.x, rocket_state->pose.orientation.y, rocket_state->pose.orientation.z, rocket_state->pose.orientation.w, 
								rocket_state->twist.angular.x, rocket_state->twist.angular.y, rocket_state->twist.angular.z,
								rocket_state->propeller_mass;
						
}


Eigen::Matrix<double, 4,1> convertControl_SI(const Eigen::Matrix<double, 4,1>& u)
{
  Eigen::Matrix<double, 4,1> input; input << 50*u(0), 50*u(1), 1000*(u(2)+1), 50*u(3);

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

POLYMPC_FORWARD_DECLARATION(/*Name*/ control_ocp, /*NX*/ 14, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)


class control_ocp : public ContinuousOCP<control_ocp, Approximation, SPARSE>
{
public:
    ~control_ocp() = default;
    control_ocp()
    {
        Q.setZero();
        R.setZero();
        Q.diagonal() << 1.0, 1.0, 5e4,    0.2, 0.2, 5e4,   500, 500, 500, 500,  100, 100, 100,    0;
        R.diagonal() << 5e2, 5e2, 0, 5e2;
        P.setIdentity();
        P.diagonal() << 1.0, 1.0, 5e4,   0.2, 0.2, 5e4,    500, 500, 500, 500,   100, 100, 100,    0;

        xs << target_point.position.x/1000, target_point.position.y/1000, target_point.position.z/1000,
                    target_point.speed.x/1000, target_point.speed.y/1000, target_point.speed.z/1000,
                    0, 0, 0, 1,
                    0, 0, 0,
                    target_point.propeller_mass;

        us << 0.0, 0.0, 0, 0.0;
    }

    static constexpr double t_start = 0.0;
    static constexpr double t_stop  = CONTROL_HORIZON;

    Eigen::Matrix<scalar_t, 14,14> Q;
    Eigen::Matrix<scalar_t, 4,4> R;
    Eigen::Matrix<scalar_t, 14,14> P;

    Eigen::Matrix<scalar_t, 14,1> xs;
    Eigen::Matrix<scalar_t, 4,1> us;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        Eigen::Matrix<T, 4,1> input; input << 50*u(0), 50*u(1),1000*(u(2)+1) , 50*u(3);
        
        // -------------- Constant Rocket parameters ----------------------------------------
        Eigen::Matrix<T, 3, 1> J_inv; J_inv << (T)(1.97/47), (T)(1.97/47), (T)(1.0/2);
        
        T Isp = (T)213;                             // Specific Impulse in [s] 

        // -------------- Constant external parameters ------------------------------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]


        // -------------- Simulation variables -----------------------------
        T mass = (T)40 + x(13);                  // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude( x(9), x(6), x(7), x(8));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Z drag --> Big approximation: speed in Z is basically the same between world and rocket
        T drag = (T)(1e6*0.5*0.3*0.0186*1.225*x(5)*x(5)); 
        
        // Force in body frame (drag + thrust) in [N]
        Eigen::Matrix<T, 3, 1> rocket_force; rocket_force << (T)input(0), (T)input(1), (T)(input(2) - drag);

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity; gravity << (T)0, (T)0, g0*mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;  total_force = rot_matrix*rocket_force - gravity;
        std::cout << "force " << total_force.transpose() << "\n";


        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T)0.0, x(10), x(11), x(12));
        
        // X, Y force and Z torque in body frame   
        Eigen::Matrix<T, 3, 1> rocket_torque; rocket_torque << input(0), input(1), input(3);
        
        
        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is Force/mass
        xdot.segment(3,3) = (T)1e-3*total_force/mass;  

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) =  (T)0.5*(omega_quat*attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix*(rocket_torque.cwiseProduct(J_inv));

        // Mass variation is proportional to thrust
        xdot(13) = -input(2)/(Isp*g0);
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

    // Create path publisher
    ros::Publisher MPC_horizon_pub = n.advertise<tvc_simulator::Trajectory>("mpc_horizon", 10);

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
  rocket.init(n);
  
	// Init MPC ----------------------------------------------------------------------------------------------------------------------
	// Creates solver
	MySolver<control_ocp, osqp_solver_t> solver;
  solver.settings().max_iter = 1;
  solver.settings().line_search_max_iter = 10;
  solver.m_qp_solver.settings().max_iter = 1000;
  solver.m_qp_solver.settings().scaling = 10;


  // Input constraints
  const double inf = std::numeric_limits<double>::infinity();
  Eigen::Matrix<double, 4, 1> lbu, ubu;
  double side_thrust = 40;
  lbu << -1, -1, -1, -1;
  ubu << 1, 1, 1, 1;

  solver.upper_bound_x().tail(44) = ubu.replicate(11,1);
  solver.lower_bound_x().tail(44) = lbu.replicate(11,1);


  // State constraints
  const double eps = 1e-1;
  Eigen::Matrix<double, 14, 1> lbx, ubx;
  //lbx << -inf, -inf, 0,   -inf, -inf, 0-eps,   -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf,  0-eps;
  //ubx << inf,   inf, inf,  inf,  inf, 330+eps,  0.183+eps,  0.183+eps,  0.183+eps,  1+eps,    inf,  inf,  inf,  3.1+eps;
  
  lbx << -inf, -inf, -inf,   -inf, -inf, -inf,   -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf,     0-eps;
  ubx << inf,  inf, inf,     inf,  inf, inf,    0.183+eps,  0.183+eps,  0.183+eps,  1+eps,     inf,  inf,  inf,      inf;

  solver.upper_bound_x().head(154) = ubx.replicate(11,1);
  solver.lower_bound_x().head(154) = lbx.replicate(11,1);

  // Init solution
  solver.primal_solution().head<154>() = init_cond.replicate(11, 1);
		  
  // --------------------------------------------------------------------------------------------------------------------

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.1),
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
      solver.problem.xs <<  target_point.position.x/1000, target_point.position.y/1000, target_point.position.z/1000,
                            target_point.speed.x/1000, target_point.speed.y/1000, target_point.speed.z/1000,
                            0, 0, 0, 1, 
                            0, 0, 0,
                            target_point.propeller_mass;
    }

    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else if (current_fsm.state_machine.compare("Launch") == 0)
		{
      // Reset solver
      //solver.m_lam.setZero();
      
      solver.primal_solution().head<154>() = init_cond.replicate(11, 1);
		  solver.upper_bound_x().segment(140, 14) = init_cond;
		  solver.lower_bound_x().segment(140, 14) = init_cond;

		  // Solve problem and save solution
			double time_now = ros::Time::now().toSec();
		  solver.solve();
//			ROS_INFO("T= %.2f ms, st: %d, iter: %d",  1000*(ros::Time::now().toSec()-time_now), solver.info().status.value, solver.m_qp_solver.m_info.iter);

		 

		  // Get state and control solution
		  Eigen::Matrix<double, 14, 11> trajectory; trajectory << Eigen::Map<Eigen::Matrix<double, 14, 11>>(solver.primal_solution().head(154).data(), 14, 11);
		  Eigen::Matrix<double, 4, 11> control_MPC_full; control_MPC_full << Eigen::Map<Eigen::Matrix<double, 4, 11>>(solver.primal_solution().tail(44).data(), 4, 11);

			//std::cout << control_MPC_full << "\n";

			Eigen::Matrix<double, 4, 1> control_MPC;
			control_MPC =  control_MPC_full.col( control_MPC_full.cols()-1 );
			//ROS_INFO("Fx: %f, Fy: %f, Fz: %f, Mx: %f \n",  control_MPC[0], control_MPC[1], control_MPC[2], control_MPC[3]);
			//Eigen::Matrix<double, 4,1> input; input << 100*control_MPC(0), 100*control_MPC(1), 1000*(control_MPC(2)+1), 50*control_MPC(3);
			
			Eigen::Matrix<double, 4,1> input = convertControl_SI(control_MPC);

			// -------------------------------------------------------------------------------------------------------------------------


      // Simple P controller. Thrust in z is used to reach apogee, Thrust in x and y is used to keep vertical orientation
			thrust_force.z = (target_point.position.z - current_state.pose.position.z)*100;

      thrust_force.x = -current_state.pose.orientation.x*900/rocket.dry_CM;
      thrust_force.y = -current_state.pose.orientation.y*900/rocket.dry_CM;

			thrust_force.x = input[0];
			thrust_force.y = input[1];
			thrust_force.z = input[2];


      if(thrust_force.z > rocket.maxThrust) thrust_force.z = rocket.maxThrust;
      if(thrust_force.z < rocket.minThrust) thrust_force.z = rocket.minThrust;
      
      if(thrust_force.x > 100) thrust_force.x = 100;
      if(thrust_force.x < -100) thrust_force.x = -100;
      if(thrust_force.y > 100) thrust_force.y = 100;
      if(thrust_force.y < -100) thrust_force.y = -100;

      // Torque in X and Y is defined by thrust in X and Y. Torque in Z is free variable
      thrust_torque.x = thrust_force.x*1.97; 
      thrust_torque.y = thrust_force.y*1.97;
			thrust_torque.z = input[3];

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;

			tvc_simulator::Trajectory trajectory_msg;
                for(int i = 0; i<11;i++){
                    geometry_msgs::Point point;
                    point.x = 1000*trajectory(0, i);
                    point.y = 1000*trajectory(1, i);
                    point.z = 1000*trajectory(2, i);
                    trajectory_msg.trajectory.push_back(point);
                }

                MPC_horizon_pub.publish(trajectory_msg);


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
