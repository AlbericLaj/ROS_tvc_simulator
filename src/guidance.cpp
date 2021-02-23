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

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/box_admm.hpp"
#include "solvers/admm.hpp"
#include "solvers/osqp_interface.hpp"

#include "utils/helpers.hpp"
#include "control/mpc_wrapper.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;

class Rocket
{
  public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    std::vector<float> maxThrust{0, 0, 0};
    std::vector<float> minThrust{0, 0, 0};

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};
    std::vector<float> drag_coeff = {0, 0, 0};

  void init(ros::NodeHandle n)
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

    float rho_air = 1.225;
    drag_coeff[0] = 0.5*rho_air*surface[0]*Cd[0];
    drag_coeff[1] = 0.5*rho_air*surface[1]*Cd[1];
    drag_coeff[2] = 0.5*rho_air*surface[2]*Cd[2];
  }

};

Rocket rocket;



// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

#define N_POINT POLY_ORDER*NUM_SEG // Number of collocation points

#define HORIZON_LENGTH 30.0 // in seccnds

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ guidance_ocp, /*NX*/ 7, /*NU*/ 3, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)
using namespace Eigen;

class guidance_ocp : public ContinuousOCP<guidance_ocp, Approximation, SPARSE>
{
public:
    ~guidance_ocp() = default;

    static constexpr double t_start = 0;
    static constexpr double t_stop  = HORIZON_LENGTH;

    Eigen::DiagonalMatrix<scalar_t, 7> Q{1.0, 1.0, 15, 0.2, 0.2, 0.5, 0};
    Eigen::DiagonalMatrix<scalar_t, 3> R{0.1, 0.1, 0};
    Eigen::DiagonalMatrix<scalar_t, 7> QN{1.0, 1.0, 100, 0.2, 0.2, 0.5, 0};

    Eigen::Matrix<scalar_t, 7,1> xs;
    Eigen::Matrix<scalar_t, 3,1> us{0.0, 0.0, 5*40*9.81};

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        std::cout << x << "\n" << u << "\n\n";
        // -------------- Simulation variables -----------------------------
        T mass = (T)rocket.dry_mass + x(6);                   // Instantaneous mass of the rocekt in [kg]
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]


        // Force in 3D in [N]: Drag, Gravity and Thrust --------------------

        // Drag is drag_coefficient*speed²
        Eigen::Matrix<T, 3, 1> drag; drag << (T)rocket.drag_coeff[0], (T)rocket.drag_coeff[1], (T)rocket.drag_coeff[2];
        drag = drag.cwiseProduct(x.segment(3,3).cwiseProduct(x.segment(3,3)));

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T)0, (T)0, g0*mass;

        // Total force in 3D in [N]
        Eigen::Matrix<T, 3, 1> total_force;
        total_force = u - drag - gravity;

        // -------------- Differential equation ---------------------

        // Position variation is mass
        xdot.head(3) = x.segment(3,3);

        // Speed variation is Force/mass
        xdot.segment(3,3) = total_force/mass;  

        // Mass variation is proportional to thrust
        xdot(6) = -u(2)/((T)rocket.Isp*g0);
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                   const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept
    {
        Eigen::Matrix<T,7,7> Qm = Q.toDenseMatrix().template cast<T>();
        Eigen::Matrix<T,3,3> Rm = R.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,7,1> x_error = x - xs.template cast<T>();
        Eigen::Matrix<T,3,1> u_error = u - us.template cast<T>();
        

        lagrange = x_error.dot(Qm * x_error) + u_error.dot(Rm * u_error);
    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept
    {
        Eigen::Matrix<T,7,7> Qm = QN.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,7,1> x_error = x - xs.template cast<T>();
                
        mayer = x_error.dot(Qm * x_error);    }
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

    EIGEN_STRONG_INLINE Problem& get_problem() noexcept { return this->problem; }

    /** change Hessian regularization to non-default*/
    EIGEN_STRONG_INLINE void hessian_regularisation_dense_impl(Eigen::Ref<nlp_hessian_t> lag_hessian) noexcept
    { //Three methods: adding a constant identity matrix, estimating eigen values with Greshgoring circles, Eigen Value Decomposition        
      const int n = this->m_H.rows();                
      
      /**Regularize by the estimation of the minimum negative eigen value--does not work with inexact Hessian update(matrix is already PSD)*/
      scalar_t aii, ri;
      for (int i = 0; i < n; i++)
      {
        aii = lag_hessian(i,i);
        ri  = (lag_hessian.col(i).cwiseAbs()).sum() - abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal
        if (aii - ri <= 0) {lag_hessian(i,i) += (ri - aii) + scalar_t(0.01);} //All Greshgorin discs are in the positive half               
        }
    }
    EIGEN_STRONG_INLINE void hessian_regularisation_sparse_impl(nlp_hessian_t& lag_hessian) noexcept
    {//Three methods: adding a constant identity matrix, estimating eigen values with Gershgorin circles, Eigen Value Decomposition
        const int n = this->m_H.rows(); //132=m_H.toDense().rows()        
        
        /**Regularize by the estimation of the minimum negative eigen value*/
        scalar_t aii, ri;
        for (int i = 0; i < n; i++)
        {
            aii = lag_hessian.coeffRef(i, i);
            ri = (lag_hessian.col(i).cwiseAbs()).sum() - abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal            
            if (aii - ri <= 0)
                lag_hessian.coeffRef(i, i) += (ri - aii) + 0.001;//All Gershgorin discs are in the positive half
        }
    }


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




// Global variables
using admm = boxADMM<guidance_ocp::VAR_SIZE, guidance_ocp::NUM_EQ, guidance_ocp::scalar_t,
                guidance_ocp::MATRIXFMT, linear_solver_traits<guidance_ocp::MATRIXFMT>::default_solver>;

using osqp_solver_t = polympc::OSQP<guidance_ocp::VAR_SIZE, guidance_ocp::NUM_EQ, guidance_ocp::scalar_t>;

// Creates solver
using mpc_t = MPC<guidance_ocp, MySolver, admm>;
mpc_t mpc;

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

// Global variable with last received rocket state
tvc_simulator::State current_state;

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}

// Service function: send back waypoint at requested time
bool sendWaypoint(tvc_simulator::GetWaypoint::Request &req, tvc_simulator::GetWaypoint::Response &res)
{
  Eigen::Matrix<double, 7, 1> next_waypoint; next_waypoint =  mpc.solution_x_at((float)(req.target_time-current_fsm.time_now)); 
  
  res.target_point.position.x = next_waypoint(0);
  res.target_point.position.y = next_waypoint(1);
  res.target_point.position.z = next_waypoint(2);

  res.target_point.speed.x = next_waypoint(3);
  res.target_point.speed.y = next_waypoint(4);
  res.target_point.speed.z = next_waypoint(5);

  res.target_point.propeller_mass = next_waypoint(6);

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
	
	// Create waypoint publisher
	ros::Publisher waypoint_pub = n.advertise<tvc_simulator::Waypoint>("waypoint_pub", 10);

	// Subscribe to state message from simulation
  ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);
	
	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

    // Initialize rocket class with useful parameters
  rocket.init(n);

	// Init MPC ----------------------------------------------------------------------------------------------------------------------
	
  mpc.settings().max_iter = 10;
  mpc.settings().line_search_max_iter = 10;
  //mpc.m_solver.settings().max_iter = 1000;
  //mpc.m_solver.settings().scaling = 10;


  // Input constraints
  const double inf = std::numeric_limits<double>::infinity();
  mpc_t::control_t lbu; 
  mpc_t::control_t ubu; 

  lbu << rocket.minThrust[0], rocket.minThrust[1], rocket.minThrust[2]; // lower bound on control
	ubu << rocket.maxThrust[0], rocket.maxThrust[1], rocket.maxThrust[2]; // lower bound on control
  
  //lbu << -inf, -inf, -inf;
  //ubu <<  inf,  inf,  inf;
  mpc.control_bounds(lbu, ubu);


  // State constraints
  const double eps = 1e-1;
  mpc_t::state_t lbx; 
  mpc_t::state_t ubx; 
  
  lbx << -inf, -inf, 0,       -inf, -inf, 0-eps,      0-eps;
  ubx <<  inf,  inf, inf,      inf,  inf, 330+eps,    rocket.propellant_mass+eps;
  
  //lbx << -inf, -inf, -inf,   -inf, -inf, -inf,   -inf, -inf, -inf, -inf,   -inf, -inf, -inf,     -inf;
  //ubx << inf,  inf, inf,     inf,  inf, inf,    inf,  inf,  inf,  inf,     inf,  inf,  inf,      inf;
  mpc.state_bounds(lbx, ubx);
  

  // Initial state
  mpc_t::state_t x0;
  x0 << 0, 0, 0,
        0, 0, 0,
        rocket.propellant_mass;
  mpc.x_guess(x0.replicate(7,1));	

  // Target point
  mpc.ocp().xs << rocket.target_apogee[0], rocket.target_apogee[1], rocket.target_apogee[2],      0.0, 0.0, 0.0,        0.0;
	
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
        x0 <<   current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
                current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
                current_state.propeller_mass;

        mpc.initial_conditions(x0);
        mpc.x_guess(x0.replicate(7,1));	

        // Solve problem and save solution
        double time_now = ros::Time::now().toSec();
        mpc.solve();
        ROS_INFO("Gdc T= %.2f ms, st: %d, iter: %d",  1000*(ros::Time::now().toSec()-time_now), mpc.info().status.value,  mpc.info().iter);
        std::cout << mpc.solution_u_at(0) << "\n";

				int i;
				for(i=0; i<N_POINT; i++)
				{
          Eigen::Matrix<double, 7, 1> guidance_point; guidance_point =  mpc.solution_x_at(i);
          tvc_simulator::Waypoint waypoint;
  
          waypoint.position.x = guidance_point(0);
          waypoint.position.y = guidance_point(1);
          waypoint.position.z = guidance_point(2);

          waypoint.speed.x = guidance_point(3);
          waypoint.speed.y = guidance_point(4);
          waypoint.speed.z = guidance_point(5);

          waypoint.propeller_mass = guidance_point(6);

          waypoint.time = 0.5*(2*current_fsm.time_now + HORIZON_LENGTH) - 0.5*(HORIZON_LENGTH)*cos(3.14159*(2*i+1)/(2*N_POINT));

          waypoint_pub.publish(waypoint);
				}

			}

      else if (current_fsm.state_machine.compare("Coast") == 0)
		  {
      }
		
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}