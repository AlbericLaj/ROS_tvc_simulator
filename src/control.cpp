#include "ros/ros.h"

#include <ros/package.h>

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

#define CONTROL_HORIZON 2 // In seconds

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

using namespace Eigen;


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

    std::vector<float> full_thrust;
    std::vector<float> full_thrust_time;


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

      load_motor();
    }

    void load_motor()
    {
      std::string path = ros::package::getPath("tvc_simulator") + "/config/motor_file.txt";

      std::string line;
      std::ifstream myfile (path);
      if (myfile.is_open())
      {
        while ( getline (myfile,line) )
        {
          int separator = line.find("\t");
          std::string time_string = line.substr(0, separator);
          std::string thrust_string = line.substr(separator+1, line.length()-separator);

          full_thrust_time.push_back(std::stof(time_string));
          full_thrust.push_back(std::stof(thrust_string));
        }
        myfile.close();

      }
      else{ROS_WARN("Didn't find motor file");}
    }

    float get_full_thrust(float time_thrust)
    {
      int i;
      for(i = 0; i<full_thrust_time.size(); i++)
      {
        if(time_thrust < full_thrust_time[i])
        {
          break;
        }
      }
      i--;
      
      float interp_thrust;
      if (time_thrust < full_thrust_time.back())
        interp_thrust = (full_thrust[i] + (time_thrust-full_thrust_time[i])/(full_thrust_time[i+1]-full_thrust_time[i])*(full_thrust[i+1]-full_thrust[i]));

      else 
        interp_thrust = full_thrust_time.back();

      if(time_thrust < 0)
        interp_thrust = 0;

      return interp_thrust;
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
  Eigen::Matrix<double, 4,1> input; input << rocket.maxThrust[0]*u(0), rocket.maxThrust[1]*u(1), 0.5*((rocket.maxThrust[2]-rocket.minThrust[2])*u(2) + rocket.maxThrust[2]+rocket.minThrust[2] ), rocket.maxTorque*u(3);
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

    static constexpr double t_start = 0.0;
    static constexpr double t_stop  = CONTROL_HORIZON;

    Eigen::DiagonalMatrix<scalar_t, 14> Q{1.0, 1.0, 5e4,    0.2, 0.2, 100,   5000, 5000, 5000, 5000,  500, 500, 500,    0};
    Eigen::DiagonalMatrix<scalar_t, 4> R{5, 5, 1000, 5};
    Eigen::DiagonalMatrix<scalar_t, 14> QN{1.0, 1.0, 5e4,   0.2, 0.2, 100,    5000, 5000, 5000, 5000,   500, 500, 500,    0};

    Eigen::Matrix<scalar_t, 14,1> xs;
    Eigen::Matrix<scalar_t, 4,1> us{0.0, 0.0, 0, 0.0};

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        Eigen::Matrix<T, 4,1> input; input << rocket.maxThrust[0]*u(0), rocket.maxThrust[1]*u(1), 0.5*((rocket.maxThrust[2]-rocket.minThrust[2])*u(2) + rocket.maxThrust[2]+rocket.minThrust[2] ), rocket.maxTorque*u(3);

        // -------------- Simulation parameters -------------- -------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]

        Eigen::Matrix<T, 3, 1> J_inv; J_inv << (T)rocket.J_inv[0], (T)rocket.J_inv[1], (T)rocket.J_inv[2]; // Cemter of mass divided by axes inertia

        // -------------- Simulation variables -----------------------------
        T mass = (T)rocket.dry_mass + x(6);                  // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude( x(9), x(6), x(7), x(8));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Z drag --> Big approximation: speed in Z is basically the same between world and rocket
        T drag = (T)(1e6*rocket.drag_coeff[2]*x(5)*x(5)); 
        
        // Force in body frame (drag + thrust) in [N]
        Eigen::Matrix<T, 3, 1> rocket_force; rocket_force << (T)input(0), (T)input(1), (T)(input(2) - drag);

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity; gravity << (T)0, (T)0, g0*mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;  total_force = rot_matrix*rocket_force - gravity;

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
        xdot(13) = -input(2)/((T)rocket.Isp*g0);
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                   const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept
    {                 
        Eigen::Matrix<T,14,14> Qm = Q.toDenseMatrix().template cast<T>();
        Eigen::Matrix<T,4,4> Rm = R.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,14,1> x_error = x - xs.template cast<T>();
        Eigen::Matrix<T,4,1> u_error = u - us.template cast<T>();
        

        lagrange = x_error.dot(Qm * x_error) + u_error.dot(Rm * u_error);

    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept
    {
        Eigen::Matrix<T,14,14> Qm = QN.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,14,1> x_error = x - xs.template cast<T>();
                
        mayer = x_error.dot(Qm * x_error);
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




// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------




















int main(int argc, char **argv)
{
  using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
                 control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
                 
  using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;


	// Init ROS control node
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
	using mpc_t = MPC<control_ocp, MySolver, osqp_solver_t>;
	mpc_t mpc;
	
  mpc.settings().max_iter = 1;
  mpc.settings().line_search_max_iter = 10;
  //mpc.m_solver.qp_settings().max_iter = 100;

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
  
  lbx << -inf, -inf, 0,     -inf, -inf, 0-eps,     -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf,     0-eps;
  ubx <<  inf,  inf, inf,    inf,  inf, 330+eps,    0.183+eps,  0.183+eps,  0.183+eps,  1+eps,    inf,  inf,  inf,     rocket.propellant_mass+eps;
  
  //lbx << -inf, -inf, -inf,   -inf, -inf, -inf,   -inf, -inf, -inf, -inf,   -inf, -inf, -inf,     -inf;
  //ubx << inf,  inf, inf,     inf,  inf, inf,    inf,  inf,  inf,  inf,     inf,  inf,  inf,      inf;
  mpc.state_bounds(lbx, ubx);
  

  // Initial state
  mpc_t::state_t x0;
  x0 << 0, 0, 0,
        0, 0, 0,
        0, 0, 0, 1, 
        0, 0, 0,
        rocket.propellant_mass;
  mpc.x_guess(x0.replicate(14,1));	

  // Variables to track performance over whole simulation
	std::vector<float> average_time;
  std::vector<int> average_status;

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent&) 
	{
    // Get current FSM and time
    if(client_fsm.call(srv_fsm))
    {
      current_fsm = srv_fsm.response.fsm;
    }
    /*
    // Init state trajectory guess from current state and guidance trajectory
    mpc_t::traj_state_t x_init;
    mpc_t::traj_control_t u_init;
    mpc_t::state_t target_point;
    mpc_t::control_t target_control;
    for(int i = 0; i<mpc.ocp().NUM_NODES; i++)
    {
      srv_waypoint.request.target_time = current_fsm.time_now + mpc.time_grid(i);
      if(client_waypoint.call(srv_waypoint))
      {
        
        target_point << srv_waypoint.response.target_point.position.x/1000, srv_waypoint.response.target_point.position.y/1000, srv_waypoint.response.target_point.position.z/1000,
                        srv_waypoint.response.target_point.speed.x/1000, srv_waypoint.response.target_point.speed.y/1000, srv_waypoint.response.target_point.speed.z/1000,
                        current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z,current_state.pose.orientation.w, 
							          current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z,
                        srv_waypoint.response.target_point.propeller_mass;


        target_control <<  0, 0, 
                          (2*srv_waypoint.response.target_point.thrust - rocket.maxThrust[2] - rocket.minThrust[2])/(rocket.maxThrust[2]-rocket.minThrust[2]),
                           0;

        u_init.segment(i*4, 4) = target_control;
        x_init.segment(i*14, 14) = target_point;
      }
    }
    mpc.u_guess(u_init);
    mpc.x_guess(x_init);

    mpc.ocp().xs <<  target_point; // last trajectory point is also our target
    mpc.ocp().us << target_control;

    */
    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

    else if (current_fsm.state_machine.compare("Rail") == 0)
    {
      thrust_force.x = 0;
      thrust_force.y = 0;
      thrust_force.z = rocket.get_full_thrust(current_fsm.time_now);

      thrust_torque.x = 0;
      thrust_torque.y = 0;
      thrust_torque.z = 0;

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;
    }

		else if (current_fsm.state_machine.compare("Launch") == 0)
		{
      x0 <<   current_state.pose.position.x/1000, current_state.pose.position.y/1000, current_state.pose.position.z/1000,
							current_state.twist.linear.x/1000, current_state.twist.linear.y/1000, current_state.twist.linear.z/1000,
							current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z,current_state.pose.orientation.w, 
							current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z,
							current_state.propeller_mass;

      mpc.initial_conditions(x0);
      rocket.update_CM(x0(13));

		  // Solve problem and save solution
			double time_now = ros::Time::now().toSec();
		  //mpc.solve();
      //time_now = 1000*(ros::Time::now().toSec()-time_now);

			//ROS_INFO("Ctr T= %.2f ms, st: %d, iter: %d", time_now , mpc.info().status.value,  mpc.info().iter);
      average_status.push_back(mpc.info().status.value);
      average_time.push_back(time_now);

		  // Get state and control solution
			Eigen::Matrix<double, 4, 1> control_MPC;
			control_MPC =  mpc.solution_u_at(0);
			
			Eigen::Matrix<double, 4,1> input = convertControl_SI(control_MPC);
			//ROS_INFO("Fx: %f, Fy: %f, Fz: %f, Mx: %f \n",  input[0], input[1], input[2], input[3]);


      // Simple P controller. Thrust in z is used to reach apogee, Thrust in x and y is used to keep vertical orientation
			thrust_force.z = rocket.get_full_thrust(current_fsm.time_now);
      thrust_force.x = 0;
      thrust_force.y = 0;

      // Apply MPC control
			//thrust_force.x = input[0];
			//thrust_force.y = input[1];
			//thrust_force.z = input[2];
      //thrust_torque.z = input[3];

      // Limit input between min and max
      if(thrust_force.z > rocket.maxThrust[2]) thrust_force.z = rocket.maxThrust[2];
      if(thrust_force.z < rocket.minThrust[2]) thrust_force.z = rocket.minThrust[2];
      
      if(thrust_force.x > rocket.maxThrust[0]) thrust_force.x = rocket.maxThrust[0];
      if(thrust_force.x < rocket.minThrust[0]) thrust_force.x = rocket.minThrust[0];

      if(thrust_force.y > rocket.maxThrust[1]) thrust_force.y = rocket.maxThrust[1];
      if(thrust_force.y < rocket.minThrust[1]) thrust_force.y = rocket.maxThrust[1];

      // Torque in X and Y is defined by thrust in X and Y. Torque in Z is free variable
      thrust_torque.x = thrust_force.x*rocket.total_CM; 
      thrust_torque.y = thrust_force.y*rocket.total_CM;

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;
      //std::cout << control_law << "\n";

      // Send optimal trajectory computed by control. Send only position for now
			tvc_simulator::Trajectory trajectory_msg;
      for(int i = 0; i<mpc.ocp().NUM_NODES ;i++){
          tvc_simulator::Waypoint point;
          point.position.x = 1000*mpc.solution_x_at(i)[0];
          point.position.y = 1000*mpc.solution_x_at(i)[1];
          point.position.z = 1000*mpc.solution_x_at(i)[2];
          trajectory_msg.trajectory.push_back(point);
      }

      MPC_horizon_pub.publish(trajectory_msg);

		}

		else if (current_fsm.state_machine.compare("Coast") == 0)
		{
      // Slow down control to 10s period for reducing useless computation
      control_thread.setPeriod(ros::Duration(10.0));

      thrust_force.x = 0;
      thrust_force.y = 0;
      thrust_force.z = 0;

      thrust_torque.x = 0;
      thrust_torque.y = 0;
      thrust_torque.z = 0;

			control_law.force = thrust_force;
			control_law.torque = thrust_torque;

      std::cout << "Average time: " << (std::accumulate(average_time.begin(), average_time.end(), 0.0))/average_time.size()  << "ms | Average status: " << (std::accumulate(average_status.begin(), average_status.end(), 0.0))/average_status.size()  << "\n";
    }
	
		control_pub.publish(control_law);
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}
