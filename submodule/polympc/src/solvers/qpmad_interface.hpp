#ifndef QPMAD_INTERFACE_HPP
#define QPMAD_INTERFACE_HPP

#ifdef POLYMPC_FOUND_QPMAD

#include "qp_base.hpp"
#include "qpmad/solver.h"

namespace polympc {

template<int N, int M, typename Scalar = double, int MatrixType = DENSE>
class QPMAD : public QPBase<QPMAD<N, M, Scalar, MatrixType>, N, M, Scalar, MatrixType>
{
    using Base = QPBase<QPMAD<N, M, Scalar, MatrixType>, N, M, Scalar, MatrixType>;
    using qp_var_t        = typename Base::qp_var_t;
    using qp_dual_t       = typename Base::qp_dual_t;
    using qp_dual_a_t     = typename Base::qp_dual_a_t;
    using qp_constraint_t = typename Base::qp_constraint_t;
    using qp_hessian_t    = typename Base::qp_hessian_t;
    using scalar_t        = typename Base::scalar_t;
    using kkt_mat_t       = typename Base::kkt_mat_t;
    using kkt_vec_t       = typename Base::kkt_vec_t;
    using linear_solver_t = typename Base::linear_solver_t;

public:
    /** constructor */
    QPMAD() : Base()
    {
        EIGEN_STATIC_ASSERT(MatrixType == DENSE, "QPMAD_Interface: QPMAD does not support sparse matrices \n");
        EIGEN_STATIC_ASSERT((std::is_same<Scalar, double>::value == true), "QPMAD_Interface: QPMAD only supports 'double' precision \n");

        set_qpmad_settings();
    }
    ~QPMAD() = default;

    void set_qpmad_settings() noexcept
    {
        /** set the settings */
        qpmad_parameters.max_iter_     = this->m_settings.max_iter;
        qpmad_parameters.tolerance_    = this->m_settings.eps_abs;
        qpmad_parameters.hessian_type_ = (qpmad::SolverParameters::HessianType)this->m_settings.hessian_type;
    }

    /** solve */
    status_t solve_impl(const Eigen::Ref<const qp_hessian_t>& H, const Eigen::Ref<const qp_var_t>& h, const Eigen::Ref<const qp_constraint_t>& A,
                        const Eigen::Ref<const qp_dual_a_t>& Alb, const Eigen::Ref<const qp_dual_a_t>& Aub,
                        const Eigen::Ref<const qp_var_t>& xlb, const Eigen::Ref<const qp_var_t>& xub) noexcept
    {
        set_qpmad_settings();

        /** @bug : temporary fix until the new interface is released */
        qpmad::QPVector primal  = Eigen::Map<qpmad::QPVector>(this->m_x.data(), this->m_x.rows());
        qpmad::QPMatrix hessian(H); //Eigen::Map<qpmad::QPMatrix>(H.derived().data(), N, N);

        qpmad::Solver::ReturnStatus status = qpmad_solver.solve(primal, hessian, h, xlb, xub, A, Alb, Aub, qpmad_parameters);

        // temporary solution
        this->m_x = primal;

        switch (status)
        {
            case qpmad::Solver::OK : {this->m_info.status = status_t::SOLVED; return status_t::SOLVED;}
            case qpmad::Solver::MAXIMAL_NUMBER_OF_ITERATIONS : {this->m_info.status = status_t::MAX_ITER_EXCEEDED; return status_t::MAX_ITER_EXCEEDED;}
            case qpmad::Solver::INFEASIBLE_EQUALITY : {this->m_info.status = status_t::INFEASIBLE; return status_t::INFEASIBLE;}
            case qpmad::Solver::INFEASIBLE_INEQUALITY : {this->m_info.status = status_t::INFEASIBLE; return status_t::INFEASIBLE;}
            case qpmad::Solver::INCONSISTENT : {this->m_info.status = status_t::INCONSISTENT; return status_t::INCONSISTENT;}
            default: return status_t::UNSOLVED;
        }
    }

    status_t solve_impl(const Eigen::Ref<const qp_hessian_t>& H, const Eigen::Ref<const qp_var_t>& h, const Eigen::Ref<const qp_constraint_t>& A,
                        const Eigen::Ref<const qp_dual_a_t>& Alb, const Eigen::Ref<const qp_dual_a_t>& Aub,
                        const Eigen::Ref<const qp_var_t>& xlb, const Eigen::Ref<const qp_var_t>& xub,
                        const Eigen::Ref<const qp_var_t>& x_guess, const Eigen::Ref<const qp_dual_t>& y_guess) noexcept
    {
        set_qpmad_settings();
        this->m_x = x_guess;

        /** @bug : temporary fix until the new interface is released */
        qpmad::QPVector primal  = Eigen::Map<qpmad::QPVector>(this->m_x.data(), this->m_x.rows());
        qpmad::QPMatrix hessian(H);
        qpmad::Solver::ReturnStatus status = qpmad_solver.solve(primal, hessian, h, xlb, xub, A, Alb, Aub, qpmad_parameters);

        this->m_x = primal;

        switch (status)
        {
            case qpmad::Solver::OK : {this->m_info.status = status_t::SOLVED; return status_t::SOLVED;}
            case qpmad::Solver::MAXIMAL_NUMBER_OF_ITERATIONS : {this->m_info.status = status_t::MAX_ITER_EXCEEDED; return status_t::MAX_ITER_EXCEEDED;}
            case qpmad::Solver::INFEASIBLE_EQUALITY : {this->m_info.status = status_t::INFEASIBLE; return status_t::INFEASIBLE;}
            case qpmad::Solver::INFEASIBLE_INEQUALITY : {this->m_info.status = status_t::INFEASIBLE; return status_t::INFEASIBLE;}
            case qpmad::Solver::INCONSISTENT : {this->m_info.status = status_t::INCONSISTENT; return status_t::INCONSISTENT;}
            default: return status_t::UNSOLVED;
        }
    }


private:
    qpmad::Solver qpmad_solver;
    qpmad::SolverParameters qpmad_parameters;
};

} // polympc namespace

#endif // POLYMPC_FOUND_QPMAD

#endif // QPMAD_INTERFACE_HPP
