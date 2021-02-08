#include "solvers/sqp_base.hpp"
#include "polynomials/ebyshev.hpp"
//#include "control/ode_collocation.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"
#include <iomanip>
#include <iostream>
#include <chrono>
#include "control/simple_robot_model.hpp"
#include "solvers/box_admm.hpp"


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

#define test_POLY_ORDER 5
#define test_NUM_SEG    2
#define test_NUM_EXP    1

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<test_POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, test_NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ cstr_ocp, /*NX*/ 4, /*NU*/ 2, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)

using namespace Eigen;

class cstr_ocp : public ContinuousOCP<cstr_ocp, Approximation, SPARSE>
{
public:
    ~cstr_ocp() = default;
    cstr_ocp()
    {
        Q.setZero();
        R.setZero();
        Q.diagonal() << 0.2, 1.0, 0.5, 0.2;
        R.diagonal() << 0.5, 5.0 * 1.0e-7;
        P << 1.4646778374584373, 0.6676889516721198, 0.35446715117028615, 0.10324422005086348,
             0.6676889516721198, 1.407812935783267,	 0.17788030743777067, 0.050059833257226405,
             0.3544671511702861, 0.1778803074377706, 0.6336052592712396,  0.01110329497282364,
             0.1032442200508634, 0.05005983325722643, 0.011103294972823655,	0.229412393739723;

        xs << 2.1402105301746182e00, 1.0903043613077321e00, 1.1419108442079495e02, 1.1290659291045561e02;
        us << 14.19, -1113.50;
    }

    static constexpr double t_start = 0.0;
    static constexpr double t_stop  = 150.0;

    Eigen::Matrix<scalar_t, 4,4> Q;
    Eigen::Matrix<scalar_t, 2,2> R;
    Eigen::Matrix<scalar_t, 4,4> P;

    Eigen::Matrix<scalar_t, 4,1> xs;
    Eigen::Matrix<scalar_t, 2,1> us;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        T c_AO = (T)5.1;
        T v_0 = (T)104.9;
        T k_w = (T)4032.0;
        T A_R = (T)0.215;
        T rho = (T)0.9342;
        T C_P = (T)3.01;
        T V_R = (T)10.0;
        T H_1 = (T)4.2;
        T H_2 = (T)-11.0;
        T H_3 = (T)-41.85;
        T m_K = (T)5.0;
        T C_PK = (T)2.0;
        T k10 =  (T)1.287e12;
        T k20 =  (T)1.287e12;
        T k30 =  (T)9.043e09;
        T E1  =  (T)-9758.3;
        T E2  =  (T)-9758.3;
        T E3  =  (T)-8560.0;
        T k_1 = k10 * exp(E1 / (273.15 + x(2)));
        T k_2 = k20 * exp(E2 / (273.15 + x(2)));
        T k_3 = k30 * exp(E3 / (273.15 + x(2)));
        T TIMEUNITS_PER_HOUR = (T)3600.0;

        xdot(0) = (1 / TIMEUNITS_PER_HOUR) * (u(0) * (c_AO - x(0)) - k_1 * x(0) - k_3 * x(0) * x(0));
        xdot(1) = (1 / TIMEUNITS_PER_HOUR) * (-u(0) * x(1) + k_1 * x(0) - k_2*x(1));
        xdot(2) = (1 / TIMEUNITS_PER_HOUR) * (u(0) * (v_0-x(2)) + (k_w * A_R / (rho * C_P * V_R)) *
                                                    (x(3) - x(2)) - (1 /( rho*C_P)) * (k_1*x(0) * H_1 + k_2 * x(1) * H_2 + k_3 * x(0) * x(1) * H_3));
        xdot(3) = (1 / TIMEUNITS_PER_HOUR) * ((1 / (m_K * C_PK)) * (u(1) + k_w * A_R * (x(2)-x(3))));
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

/** create solver */
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



int main(void)
{
    using admm = boxADMM<cstr_ocp::VAR_SIZE, cstr_ocp::NUM_EQ, cstr_ocp::scalar_t,
                 cstr_ocp::MATRIXFMT, linear_solver_traits<cstr_ocp::MATRIXFMT>::default_solver>;

    MySolver<cstr_ocp, admm> solver;
    solver.settings().max_iter = 10;
    solver.settings().line_search_max_iter = 10;
    Eigen::Matrix<double, 4, 1> init_cond; init_cond << 1.0, 0.5, 100.0, 100.0;
    Eigen::Matrix<double, 2, 1> lbu, ubu;
    lbu << 3.0, -9000.0;
    ubu << 35.0, 0.0;

    solver.upper_bound_x().segment(40, 4) = init_cond;
    solver.lower_bound_x().segment(40, 4) = init_cond;

    solver.upper_bound_x().tail(22) = ubu.replicate(11,1);
    solver.lower_bound_x().tail(22) = lbu.replicate(11,1);

    time_point start = get_time();
    solver.solve();
    time_point stop = get_time();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Solve status: " << solver.info().status.value << "\n";
    std::cout << "Num iterations: " << solver.info().iter << "\n";
    std::cout << "Solve time: " << std::setprecision(9) << static_cast<double>(duration.count()) << "[mc] \n";

    std::cout << "Size of the solver: " << sizeof (solver) << "\n";

    std::cout << "Solution: " << solver.primal_solution().transpose() << "\n";

    // warm started iteration
    init_cond << 1.1, 0.508, 100.5, 100.1;
    solver.upper_bound_x().segment(40, 4) = init_cond;
    solver.lower_bound_x().segment(40, 4) = init_cond;

    start = get_time();
    solver.solve();
    stop = get_time();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Solve status: " << solver.info().status.value << "\n";
    std::cout << "Num iterations: " << solver.info().iter << "\n";
    std::cout << "Solve time: " << std::setprecision(9) << static_cast<double>(duration.count()) << "[mc] \n";

    std::cout << "Solution: " << solver.primal_solution().transpose() << "\n";

    return EXIT_SUCCESS;
}