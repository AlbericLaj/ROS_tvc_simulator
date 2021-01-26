#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/State.h"
#include "tvc_simulator/Waypoint.h"

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

  Rocket(ros::NodeHandle n)
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

	init_cond << rocket_state->pose.position.x, rocket_state->pose.position.y, rocket_state->pose.position.z,
								rocket_state->twist.linear.x, rocket_state->twist.linear.y, rocket_state->twist.linear.z,
								rocket_state->pose.orientation.w, rocket_state->pose.orientation.x, rocket_state->pose.orientation.y, rocket_state->pose.orientation.z, 
								rocket_state->twist.angular.x, rocket_state->twist.angular.y, rocket_state->twist.angular.z,
								rocket_state->propeller_mass;
						
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
        Q.diagonal() << 1.0, 1.0, 50,    0.2, 0.2, 50,   5000, 5000, 5000, 5000,  100, 100, 100,    2.0;
        R.diagonal() << 0.1, 0.1, 0.3, 0.1;
        P.setIdentity();
        P.diagonal() << 1.0, 1.0, 50,   0.2, 0.2, 50,    5000, 5000, 5000, 5000,   100, 100, 100,    2.0;

				xs << target_point.position.x, target_point.position.y, target_point.position.z,
							target_point.speed.x, target_point.speed.y, target_point.speed.z,
							1, 0, 0, 0,
							0, 0, 0, 
							target_point.propeller_mass;

				//std::cout<< xs << "\n";

        //xs << 0.0, 0.0, 500.0,   0.0, 0.0, 0.0,   1.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,   0.0;
        us << 0.0, 0.0, 40*9.81, 0.0;
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
        // -------------- Constant Rocket parameters ----------------------------------------
        //Eigen::Matrix<double, 7, 1> init_cond; init_cond << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1;

        T dry_mass = (T)40;                         // Dry rocket mass in [kg]
        Eigen::Array<T, 3, 1> Inertia; Inertia << (T)47, (T)47, (T)1;
        T CM = (T)1;                                // Rocket center of mass, from nozzle, in [m]

        T Isp = (T)211;                             // Specific Impulse in [s]

        Eigen::Array<T, 3, 1> Cd; Cd << (T)1.5, (T)2.5, (T)0.7;

        T diameter = (T)15.6e-2;                    // Rocket diameter in [m]
        T length = (T)4;                            // Rocket length in [m]
        T surface_front = (T)(3.14159*(diameter/2)*(diameter/2)); // Cross-section surface in [m^2]
        T surface_side = (T)(length*diameter);      // Surface of the rocket seen from the side [m^2]

        Eigen::Array<T, 3, 1> surface; surface << surface_side, surface_side, surface_front;

        // -------------- Constant external parameters ------------------------------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]
        //T rho_air = (T)(353*pow(1-0.0000225577*x(2), 5.255)/(288.15-0.0065*x(2)));
        T rho_air = (T)1.225;



        // -------------- Simulation variables -----------------------------
        T mass = dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]
        Eigen::Array<T, 3, 1> speed; speed << (T)x(3), (T)x(4), (T)x(5); // Speed in world frame

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude((T)x(6), (T)x(7), (T)x(8), (T)x(9)); attitude.normalize();

        // Force in rocket coordinates (drag + thrust) in [N]
        T drag = (T)(0.5*Cd(2)*surface_front*rho_air*x(5)*x(5)); // Big approximation: speed in Z is basically the same between world and rocket

        Eigen::Quaternion<T> rocket_force((T)0, (T)u(0), (T)u(1), (T)(u(2) - drag)); // Drag and thrust is along Z axis of rocket, then reorientated into world coordinates
        rocket_force = attitude*rocket_force*attitude.inverse();

        //std::cout << "Speed: " << speed.transpose() << "\n";
        //std::cout << "Force: " << rocket_force.vec().transpose() << "\n\n-----------";


        // Force in world coordinate: gravity
        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T)0, (T)0, g0*mass;

        // Total force in [N] (World coordinates)
        Eigen::Matrix<T, 3, 1> total_force;  total_force << (T)rocket_force.x(), (T)rocket_force.y(), (T)rocket_force.z();
        total_force = total_force - gravity;

        // Angular acceleration in world frame in [rad/s²]
        Eigen::Quaternion<T> angular_acceleration((T)0, (T)(u(0)*CM/Inertia(0)), (T)(u(1)*CM/Inertia(1)), (T)(u(3)/Inertia(2)));
        angular_acceleration = attitude*angular_acceleration*attitude.inverse();

        // Quaternion derivative over time from angular velocity omega
        Eigen::Quaternion<T> omega((T)0.0, (T)x(10), (T)x(11), (T)x(12));
        Eigen::Quaternion<T> attitude_variation; attitude_variation = omega*attitude;


        // -------------- Differential equation ---------------------

        // Position variation is mass
        xdot(0) = x(3);
        xdot(1) = x(4);
        xdot(2) = x(5);

        // Speed variation is Force/mass
        xdot(3) = total_force(0)/mass;
        xdot(4) = total_force(1)/mass;
        xdot(5) = total_force(2)/mass;

        // Quaternion variation
        xdot(6) = 0.5*attitude_variation.w();
        xdot(7) = 0.5*attitude_variation.x();
        xdot(8) = 0.5*attitude_variation.y();
        xdot(9) = 0.5*attitude_variation.z();

        // Angular speed variation is Torque/Inertia
        xdot(10) = angular_acceleration.x();
        xdot(11) = angular_acceleration.y();
        xdot(12) = angular_acceleration.z();

        // Mass variation is proportional to thrust
        xdot(13) = -u(2)/(Isp*g0);

        //std::cout << xdot(13) << "\n";
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

  //init_cond << -0.00115844129 , 0.00152988581 ,  0.0110468581 , -0.0460468466  , 0.0604409342 ,   0.688686987   ,  1.00045167 , 0.00351486184 ,-0.00468939049, 9.37592528e-05 , -0.0333691332 ,  0.0609762074 , 0.00151707781  ,   3.07643574;

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
  Rocket rocket(n);

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.06),
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
			//std::cout << target_point;
    }

    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else if (current_fsm.state_machine.compare("Launch") == 0)
		{
		// -------------------------------------------------------------------------------------------------------------------------
    
			// Creates solver
		  MySolver<control_ocp, admm> solver;
		  solver.settings().max_iter = 1;
		  solver.settings().line_search_max_iter = 1;

		  // Input constraints
		  Eigen::Matrix<double, 4, 1> lbu, ubu;
		  double side_thrust = 40;
		  lbu << -side_thrust, -side_thrust, 0.0, -50;
		  ubu << side_thrust, side_thrust, 2000.0, 50;

		  solver.upper_bound_x().tail(44) = ubu.replicate(11,1);
		  solver.lower_bound_x().tail(44) = lbu.replicate(11,1);


		  // State constraints
		  const double inf = std::numeric_limits<double>::infinity();
		  const double eps = 1e-4;
		  Eigen::Matrix<double, 14, 1> lbx, ubx;
		  lbx << -inf, -inf, 0,   -inf, -inf, 0-eps,   -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf,  0-eps;
		  ubx << inf,   inf, inf,  inf,  inf, 330+eps,  0.183+eps,  0.183+eps,  0.183+eps,  1+eps,    inf,  inf,  inf,  3.1+eps;
		  solver.upper_bound_x().head(154) = ubx.replicate(11,1);
		  solver.lower_bound_x().head(154) = lbx.replicate(11,1);

		  // Init solution
		  solver.primal_solution().head<154>() = init_cond.replicate(11, 1);
		  solver.upper_bound_x().segment(140, 14) = init_cond;
		  solver.lower_bound_x().segment(140, 14) = init_cond;

			//std::cout << init_cond.transpose() << "\n";

		  // Solve problem and save solution
			double time_now = ros::Time::now().toSec();
		  solver.solve();
			ROS_INFO("Solver duration: %f\n",  ros::Time::now().toSec()-time_now);
		 

		  // Get state and control solution
		  Eigen::Matrix<double, 14, 11> trajectory; trajectory << Eigen::Map<Eigen::Matrix<double, 14, 11>>(solver.primal_solution().head(154).data(), 14, 11);
		  Eigen::Matrix<double, 4, 11> control_MPC_full; control_MPC_full << Eigen::Map<Eigen::Matrix<double, 4, 11>>(solver.primal_solution().tail(44).data(), 4, 11);

			//std::cout << control_MPC_full << "\n";

			Eigen::Matrix<double, 4, 1> control_MPC;
			control_MPC =  control_MPC_full.col( control_MPC_full.cols()-1 );
			ROS_INFO("Fx: %f, Fy: %f, Fz: %f, Mx: %f \n",  control_MPC[0], control_MPC[1], control_MPC[2], control_MPC[3]);

			// -------------------------------------------------------------------------------------------------------------------------


      // Simple P controller. Thrust in z is used to reach apogee, Thrust in x and y is used to keep vertical orientation
			thrust_force.z = (target_point.position.z - current_state.pose.position.z)*100;

      thrust_force.x = -current_state.pose.orientation.x*900/rocket.dry_CM;
      thrust_force.y = -current_state.pose.orientation.y*900/rocket.dry_CM;

			thrust_force.x = control_MPC[0];
			thrust_force.y = control_MPC[1];
			thrust_force.z = control_MPC[2];


      if(thrust_force.z > rocket.maxThrust) thrust_force.z = rocket.maxThrust;
      if(thrust_force.z < rocket.minThrust) thrust_force.z = rocket.minThrust;

      // Torque in X and Y is defined by thrust in X and Y. Torque in Z is free variable
      thrust_torque.x = thrust_force.x*rocket.dry_CM;
      thrust_torque.y = thrust_force.y*rocket.dry_CM;
			thrust_torque.z = control_MPC[3];

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
