#include "ros/ros.h"

#include "tvc_simulator/FSM.h"
#include "tvc_simulator/DroneState.h"
#include "tvc_simulator/Waypoint.h"
#include "tvc_simulator/Trajectory.h"

#include "tvc_simulator/DroneControl.h"
#include "geometry_msgs/Vector3.h"

#include "tvc_simulator/GetFSM.h"
#include "tvc_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#define CONTROL_HORIZON 3// In seconds

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


class Rocket {
public:
    float dry_mass;

    float maxTorque;
    float maxServo1Angle;
    float maxServo2Angle;

    float maxThrust;
    float minThrust;
    float CM_to_thrust_distance;

    std::vector<float> I{0, 0, 0};
    std::vector<float> J_inv{0, 0, 0};

    //Rocket();

    void init(ros::NodeHandle n) {
        n.getParam("/rocket/maxTorque", maxTorque);
        n.getParam("/rocket/maxThrust", maxThrust);
        n.getParam("/rocket/minThrust", minThrust);
        n.getParam("/rocket/maxServo1Angle", maxServo1Angle);
        n.getParam("/rocket/maxServo2Angle", maxServo2Angle);

        n.getParam("/rocket/dry_mass", dry_mass);

        n.getParam("/rocket/CM_to_thrust_distance", CM_to_thrust_distance);
        n.getParam("/rocket/dry_I", I);

        J_inv[0] = CM_to_thrust_distance / I[0];
        J_inv[1] = CM_to_thrust_distance / I[1];
        J_inv[2] = 1 / I[2];
    }

};

Rocket rocket;

tvc_simulator::DroneState previous_state;

// Global variable with last received rocket state
tvc_simulator::DroneState current_state;

// Global variable with last requested fsm
tvc_simulator::FSM current_fsm;

geometry_msgs::Vector3 target_apogee;

// Callback function to store last received state
void rocket_stateCallback(const tvc_simulator::DroneState::ConstPtr &rocket_state) {
    previous_state = current_state;
    current_state.pose = rocket_state->pose;
    current_state.twist = rocket_state->twist;
}


// Callback function to store last received state
void targetCallback(const geometry_msgs::Vector3 &target) {
    target_apogee = target;
}


template<typename T>
inline void unScaleControl(Eigen::Matrix<T, 4, 1> &u) {
    u(0) = rocket.maxServo1Angle * u(0);
    u(1) = rocket.maxServo1Angle * u(1);
    u(2) = 0.5 * (u(2) + 1) * (rocket.maxThrust - rocket.minThrust) + rocket.minThrust;
    u(3) = rocket.maxTorque * u(3);
}

// Poly MPC stuff ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

using namespace std;

typedef std::chrono::time_point <std::chrono::system_clock> time_point;

time_point get_time() {
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

POLYMPC_FORWARD_DECLARATION(/*Name*/ control_ocp, /*NX*/ 13, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)


#define X_COST 1e5
#define D_X_COST 1e1
#define ATT_COST 1e0
#define D_ATT_COST 1e-1

#define SERVO_COST 10000
#define THRUST_COST 100
#define TORQUE_COST 1

class control_ocp : public ContinuousOCP<control_ocp, Approximation, SPARSE> {
public:
    ~control_ocp() = default;

    static constexpr double t_start = 0.0;
    static constexpr double t_stop = CONTROL_HORIZON;

    Eigen::DiagonalMatrix<scalar_t, 13> Q{X_COST, X_COST, X_COST, D_X_COST, D_X_COST, D_X_COST, ATT_COST, ATT_COST,
                                          ATT_COST, ATT_COST, D_ATT_COST, D_ATT_COST, D_ATT_COST};
    Eigen::DiagonalMatrix<scalar_t, 4> R{SERVO_COST, SERVO_COST, THRUST_COST, TORQUE_COST};
    Eigen::DiagonalMatrix<scalar_t, 13> QN{X_COST, X_COST, X_COST, D_X_COST, D_X_COST, D_X_COST, ATT_COST, ATT_COST,
                                           ATT_COST, ATT_COST, D_ATT_COST, D_ATT_COST, D_ATT_COST};

    Eigen::Matrix<scalar_t, 13, 1> xs;
    Eigen::Matrix<scalar_t, 4, 1> us;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                              const Eigen::Ref<const parameter_t <T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref <state_t<T>> xdot) const noexcept {
        Eigen::Matrix<T, 4, 1> input;
        input << u(0), u(1), u(2), u(3);
        unScaleControl(input);

        // -------------- Simulation parameters -------------- -------------
        T g0 = (T) 9.81;                             // Earth gravity in [m/s^2]

        Eigen::Matrix<T, 3, 1> J_inv;
        // Center of mass divided by axes inertia
        J_inv << (T) rocket.J_inv[0], (T) rocket.J_inv[1], (T) rocket.J_inv[2];

        // -------------- Simulation variables -----------------------------
        T mass = (T) rocket.dry_mass;

        // Orientation of the rocket with quaternion
        Eigen::Quaternion <T> attitude(x(9), x(6), x(7), x(8));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // servomotors thrust vector rotation (see drone_interface for equivalent quaternion implementation)
        Eigen::Matrix<T, 3, 1> rocket_force;
        rocket_force << (T) input(2) * sin(input(1)),
                (T) -input(2) * cos(input(1)) * sin(input(0)),
                (T) input(2) * cos(input(0)) * cos(input(1));

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T) 0, (T) 0, g0 * mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;
        total_force = rot_matrix * rocket_force - gravity;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion <T> omega_quat((T) 0.0, x(10), x(11), x(12));

        // X, Y force and Z torque in body frame
        Eigen::Matrix<T, 3, 1> rocket_torque;
        rocket_torque << rocket_force(0), rocket_force(1), input(3);

        // -------------- Differential equations ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is Force/mass
        xdot.segment(3, 3) = (T) 1e-2 * total_force / mass;

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) = (T) 0.5 * (omega_quat * attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix * (rocket_torque.cwiseProduct(J_inv));

    }

//    template<typename T>
//    inline void inequality_constraints_impl(const state_t<T> &x, const control_t<T> &u, const parameter_t<T> &p,
//                                                    const static_parameter_t &d, const scalar_t &t, constraint_t<T> &g) const noexcept {
//        // w = x(9); x = x(6);y = x(7), x(8))
//        // (w^2 + z^2)*(x^2 + y^2) corresponds to the inclination relative to (0, 0, 1)
//        g.head(1) = (x(9)^2 + x(8)^2)*(x(6)^2 + x(7)^2) - (T) 0.0;
//    }


    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                   const Eigen::Ref<const parameter_t <T>> p,
                                   const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        Eigen::Matrix<T, 13, 13> Qm = Q.toDenseMatrix().template cast<T>();
        Eigen::Matrix<T, 4, 4> Rm = R.toDenseMatrix().template cast<T>();

        Eigen::Matrix<T, 13, 1> x_error = x - xs.template cast<T>();
        Eigen::Matrix<T, 4, 1> u_error = u - us.template cast<T>();


        lagrange = x_error.dot(Qm * x_error) + u_error.dot(Rm * u_error);

    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t <T>> x, const Eigen::Ref<const control_t <T>> u,
                                const Eigen::Ref<const parameter_t <T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        Eigen::Matrix<T, 13, 13> Qm = QN.toDenseMatrix().template cast<T>();

        Eigen::Matrix<T, 13, 1> x_error = x - xs.template cast<T>();

        mayer = x_error.dot(Qm * x_error);
    }
};


// ------ Create solver -------
template<typename Problem, typename QPSolver>
class MySolver;

template<typename Problem, typename QPSolver = boxADMM<Problem::VAR_SIZE, Problem::DUAL_SIZE, typename Problem::scalar_t>>
class MySolver : public SQPBase<MySolver<Problem, QPSolver>, Problem, QPSolver> {
public:
    using Base = SQPBase<MySolver<Problem, QPSolver>, Problem, QPSolver>;
    using typename Base::scalar_t;
    using typename Base::nlp_variable_t;
    using typename Base::nlp_hessian_t;

    EIGEN_STRONG_INLINE Problem
    &

    get_problem() noexcept { return this->problem; }

    /** change Hessian regularization to non-default*/
    EIGEN_STRONG_INLINE void hessian_regularisation_dense_impl(
            Eigen::Ref <nlp_hessian_t> lag_hessian) noexcept { //Three methods: adding a constant identity matrix, estimating eigen values with Greshgoring circles, Eigen Value Decomposition
        const int n = this->m_H.rows();

        /**Regularize by the estimation of the minimum negative eigen value--does not work with inexact Hessian update(matrix is already PSD)*/
        scalar_t aii, ri;
        for (int i = 0; i < n; i++) {
            aii = lag_hessian(i, i);
            ri = (lag_hessian.col(i).cwiseAbs()).sum() -
                 abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal
            if (aii - ri <= 0) {
                lag_hessian(i, i) += (ri - aii) + scalar_t(0.01);
            } //All Greshgorin discs are in the positive half
        }
    }

    EIGEN_STRONG_INLINE void hessian_regularisation_sparse_impl(
            nlp_hessian_t &lag_hessian) noexcept {//Three methods: adding a constant identity matrix, estimating eigen values with Gershgorin circles, Eigen Value Decomposition
        const int n = this->m_H.rows(); //132=m_H.toDense().rows()

        /**Regularize by the estimation of the minimum negative eigen value*/
        scalar_t aii, ri;
        for (int i = 0; i < n; i++) {
            aii = lag_hessian.coeffRef(i, i);
            ri = (lag_hessian.col(i).cwiseAbs()).sum() -
                 abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal
            if (aii - ri <= 0)
                lag_hessian.coeffRef(i, i) += (ri - aii) + 0.001;//All Gershgorin discs are in the positive half
        }
    }

    /** change step size selection algorithm */
    scalar_t step_size_selection_impl(const Ref<const nlp_variable_t> &p) noexcept {
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
        for (int i = 1; i < this->m_settings.line_search_max_iter; i++) {
            x_step.noalias() = alpha * p;
            x_step += this->m_x;
            this->problem.cost(x_step, this->m_p, cost_step);

            //std::cout << "i: " << i << " l1: " << this->constraints_violation(x_step) << " cost: " << cost_step << "\n";

            scalar_t phi_l1_step = cost_step + mu * this->constraints_violation(x_step);

            //std::cout << "phi before: " << phi_l1 << " after: " << phi_l1_step <<  " required diff: " << alpha * this->m_settings.eta * Dp_phi_l1 << "\n";

            if (phi_l1_step <= (phi_l1 + alpha * this->m_settings.eta * Dp_phi_l1)) {
                // accept step
                return alpha;
            } else {
                alpha = tau * alpha;
            }
        }

        return alpha;
    }

    /** change Hessian update algorithm to the one provided by ContinuousOCP*/
    EIGEN_STRONG_INLINE void
    hessian_update_impl(Eigen::Ref <nlp_hessian_t> hessian, const Eigen::Ref<const nlp_variable_t> &x_step,
                        const Eigen::Ref<const nlp_variable_t> &grad_step) noexcept {
        this->problem.hessian_update_impl(hessian, x_step, grad_step);
    }
};




// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------







#define USE_PD_CONTROLLER false

const float target_height = 2;

const float kp = 0.1;
const float kd = 0.2;

const float kp_t = 0.05;
const float kd_t = 0.005;

const float kp_z = 0.5;
const float kd_z = 20;
const float ki_z = 0.001;

const float grav_comp_ff = 21;

float int_error;

void set_PD_control_law(tvc_simulator::DroneControl &control_law) {
    // Basic PD controller for attitude and PID for altitude

    float thrust = grav_comp_ff + (target_height - current_state.pose.position.z) * kp_z -
                         (current_state.pose.position.z - previous_state.pose.position.z) * kd_z + int_error * ki_z;
    control_law.servo2 = -current_state.pose.orientation.x * kp -
                         (current_state.pose.orientation.x - previous_state.pose.orientation.x) * kd;
    control_law.servo1 = -current_state.pose.orientation.y * kp -
                         (current_state.pose.orientation.y - previous_state.pose.orientation.y) * kd;
    float torque = -current_state.pose.orientation.z * kp_t -
                         (current_state.pose.orientation.z - previous_state.pose.orientation.z) * kd_t;

    control_law.top = thrust/2 + torque/2;
    control_law.bottom = thrust/2 - torque/2;

    int_error += target_height - current_state.pose.position.z;
}


int main(int argc, char **argv) {
    using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
            control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;

    using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;


    // Init ROS time keeper node
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    // Create control publisher
    ros::Publisher control_pub = n.advertise<tvc_simulator::DroneControl>("drone_control_pub", 10);

    // Create path publisher
    ros::Publisher MPC_horizon_pub = n.advertise<tvc_simulator::Trajectory>("mpc_horizon", 10);

    // Subscribe to state message from simulation
    ros::Subscriber rocket_state_sub = n.subscribe("drone_state", 100, rocket_stateCallback);

    // Subscribe to target apogee message from simulation
    ros::Subscriber target_sub = n.subscribe("target_apogee", 100, targetCallback);

    // Setup Time_keeper client and srv variable for FSM and time synchronization
    ros::ServiceClient client_fsm = n.serviceClient<tvc_simulator::GetFSM>("getFSM");
    tvc_simulator::GetFSM srv_fsm;

    // Setup Waypoint client and srv variable for trajectory following
    ros::ServiceClient client_waypoint = n.serviceClient<tvc_simulator::GetWaypoint>("getWaypoint");
    tvc_simulator::GetWaypoint srv_waypoint;

    // Initialize control
    tvc_simulator::DroneControl control_law;
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

    mpc.settings().max_iter = 6;
    mpc.settings().line_search_max_iter = 5;

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


    //TODO change state constraints
    lbx << -inf, -inf, -1.0 / 100,
            -inf, -inf, -inf,
            -0.183 - eps, -0.183 - eps, -0.183 - eps, -1 - eps,
            -inf, -inf, -inf;

    ubx << inf, inf, inf,
            inf, inf, inf,
            0.183 + eps, 0.183 + eps, 0.183 + eps, 1 + eps,
            inf, inf, inf;

//    lbx << -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf;
//    ubx << inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf;

    mpc.state_bounds(lbx, ubx);

    // Initial state
    mpc_t::state_t x0;
    x0 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;
    mpc.x_guess(x0.replicate(13, 1));

    mpc_t::control_t target_control;
    target_control << 0, 0, 0, 0;

    mpc_t::state_t target_state;
    target_state << 0, 4.0 / 100, 10.0 / 100,
            0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0;

    // Variables to track performance over whole simulation
    std::vector<float> average_time;
    std::vector<int> average_status;
    std::vector<double> average_x_error;
    std::vector<double> average_y_error;
    std::vector<double> average_z_error;

    // Thread to compute control. Duration defines interval time in seconds
    ros::Timer control_thread = n.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent &) {

        // Get current FSM and time
        if (client_fsm.call(srv_fsm)) {
            current_fsm = srv_fsm.response.fsm;
        }

        // Init state trajectory guess from current state and guidance trajectory
        mpc_t::traj_state_t x_init;
        mpc_t::traj_control_t u_init;


        //TODO warm start
//        mpc.u_guess(u_init);
//        mpc.x_guess(x_init);

        if (client_waypoint.call(srv_waypoint)) {
            target_state << srv_waypoint.response.target_point.position.x * 1e-2, srv_waypoint.response.target_point.position.y * 1e-2, srv_waypoint.response.target_point.position.z * 1e-2,
                    srv_waypoint.response.target_point.speed.x * 1e-2, srv_waypoint.response.target_point.speed.y * 1e-2, srv_waypoint.response.target_point.speed.z * 1e-2,
                    current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z,current_state.pose.orientation.w,
                    current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

            //TODO
            target_control <<  0, 0, 0, 0;
        } else {
            target_state << target_apogee.x * 1e-2, target_apogee.y * 1e-2, target_apogee.z * 1e-2,
                    0, 0, 0,
                    0, 0, 0, 1,
                    0, 0, 0;
            target_control << 0, 0, 0, 0;
        }

        mpc.ocp().xs << target_state;
        mpc.ocp().us << target_control;

        // State machine ------------------------------------------
        if (current_fsm.state_machine.compare("Idle") == 0) {
            // Do nothing
        } else if (current_fsm.state_machine.compare("Launch") == 0) {
            x0 << current_state.pose.position.x / 100, current_state.pose.position.y / 100,
                    current_state.pose.position.z / 100,
                    current_state.twist.linear.x / 100, current_state.twist.linear.y / 100,
                    current_state.twist.linear.z / 100,
                    current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w,
                    current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z;

            mpc.initial_conditions(x0);

            // Solve problem and save solution
            double time_now = ros::Time::now().toSec();
            mpc.solve();
            time_now = 1000 * (ros::Time::now().toSec() - time_now);

//            ROS_INFO("Ctr T= %.2f ms, st: %d, iter: %d", time_now, mpc.info().status.value, mpc.info().iter);
            average_status.push_back(mpc.info().status.value);
            average_time.push_back(time_now);

            // Get state and control solution
            Eigen::Matrix<double, 4, 1> control_MPC;
            control_MPC = mpc.solution_u_at(0);

            unScaleControl(control_MPC);
//            ROS_INFO("servo1: %f, servo2: %f, Fz: %f, Mx: %f \n", control_MPC[0], control_MPC[1], control_MPC[2],
//                     control_MPC[3]);

            if (USE_PD_CONTROLLER || abs(control_MPC[0]) > 100 || abs(control_MPC[1]) > 100 ||
                abs(control_MPC[2]) > 100 || abs(control_MPC[3]) > 100) {
                ROS_INFO("MPC ISSUE, SWITCHED TO PD LAW \n");
                set_PD_control_law(control_law);

                //reset mpc guess
                mpc.x_guess(x0.replicate(13, 1));
                mpc.u_guess(target_control.replicate(13, 1));
            } else {
                // Apply MPC control
                control_law.servo1 = control_MPC[0];
                control_law.servo2 = control_MPC[1];

                float thrust = control_MPC[2];
                float torque = control_MPC[3];
                control_law.top = thrust/2 + torque/2;
                control_law.bottom = thrust/2 - torque/2;
            }


            //saturate inputs
            control_law.servo1 = std::min(std::max(control_law.servo1, -rocket.maxServo1Angle), rocket.maxServo1Angle);
            control_law.servo2 = std::min(std::max(control_law.servo2, -rocket.maxServo2Angle), rocket.maxServo2Angle);
//            control_law.top = std::min(std::max(control_law.top, 0.0), 100.0); //TODO change max
//            control_law.bottom = std::min(std::max(control_law.bottom, 0.0), 100.0);

            // Send optimal trajectory computed by control. Send only position for now
            tvc_simulator::Trajectory trajectory_msg;
            for (int i = 0; i < mpc.ocp().NUM_NODES; i++) {
                tvc_simulator::Waypoint point;
                point.position.x = 100 * mpc.solution_x_at(i)[0];
                point.position.y = 100 * mpc.solution_x_at(i)[1];
                point.position.z = 100 * mpc.solution_x_at(i)[2];
                trajectory_msg.trajectory.push_back(point);
            }

            double x_error = current_state.pose.position.x - target_state(0)*100;
            average_x_error.push_back(x_error*x_error);
            double y_error = current_state.pose.position.y - target_state(1)*100;
            average_y_error.push_back(y_error*y_error);
            double z_error = current_state.pose.position.z - target_state(2)*100;
            average_z_error.push_back(z_error*z_error);

            MPC_horizon_pub.publish(trajectory_msg);

        } else if (current_fsm.state_machine.compare("Coast") == 0) {
            // Slow down control to 10s period for reducing useless computation
            control_thread.setPeriod(ros::Duration(10.0));

            control_law.servo1 = 0;
            control_law.servo2 = 0;
            control_law.top = 0;
            control_law.bottom = 0;
            std::cout << "Average time: "
                      << (std::accumulate(average_time.begin(), average_time.end(), 0.0)) / average_time.size()
                      << "ms | Average error (x, y, z): "
                      << (std::accumulate(average_x_error.begin(), average_x_error.end(), 0.0)) / average_z_error.size()
                      << ", "
                      << (std::accumulate(average_y_error.begin(), average_y_error.end(), 0.0)) / average_z_error.size()
                      << ", "
                      << (std::accumulate(average_z_error.begin(), average_z_error.end(), 0.0)) / average_z_error.size()
                      << "\n";
        }

        control_pub.publish(control_law);
        //ROS_INFO("Z force is %f", thrust_force.z);

    });

    // Automatic callback of service and publisher from here
    ros::spin();

}
