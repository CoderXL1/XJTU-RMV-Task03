#include "solver.hpp"
#include <cmath>
#include <thread>

namespace TASK03
{
    struct BallCost
    {
        BallCost(double t_, double xm_, double ym_) : t(t_), xm(xm_), ym(ym_) {}
        template <typename T>
        bool operator()(const T* const params, T* residuals) const {
            // params: [vx0, vy0, k_log, g]
            T vx0  = params[0];
            T vy0  = params[1];
            T k    = ceres::exp(params[2]); // ensure k>0
            T g    = params[3];
            // g = -g;

            T tt = T(t);

            T x_model = x0 + vx0 / k * (T(1.0) - ceres::exp(-k * tt));
            T y_model = y0 + (vy0 + g / k) / k * (T(1.0) - ceres::exp(-k * tt)) - (g / k) * tt;

            residuals[0] = x_model - T(xm);
            residuals[1] = y_model - T(ym);
            return true;
        }
        static ceres::CostFunction* get_cost_function(double t, double xm, double ym) {
            return (new ceres::AutoDiffCostFunction<BallCost, 2, 4>(
                new BallCost(t, xm, ym)));
        }
        double t, xm, ym;
    };
    // Analytic (non-AutoDiff) cost function: 2 residuals, 4 parameters
    struct BallCostAnalytic : public ceres::SizedCostFunction<2, 4>
    {
        BallCostAnalytic(double t_, double xm_, double ym_) : t(t_), xm(xm_), ym(ym_) {}

        virtual bool Evaluate(double const* const* params,
                              double* residuals,
                              double** jacobians) const override
        {
            const double* p = params[0];
            const double vx0 = p[0];
            const double vy0 = p[1];
            const double k_log = p[2];
            const double g = p[3];
            const double k = std::exp(k_log);
            const double exp_minus = std::exp(-k * t);
            const double A = 1.0 - exp_minus;

            // Models
            const double x_model = x0 + vx0 / k * A;
            const double B = vy0 / k + g / (k * k);
            const double y_model = y0 + B * A - (g / k) * t;

            residuals[0] = x_model - xm;
            residuals[1] = y_model - ym;

            if (jacobians != nullptr && jacobians[0] != nullptr) {
                double* J = jacobians[0]; // m x n in row-major (m=2, n=4)
                // r1 derivatives
                const double dr1_dvx0 = A / k;
                const double dr1_dvy0 = 0.0;
                // dr1/dk_log = (d r1 / d k) * k
                const double dA_dk = t * exp_minus;
                const double dr1_dk = vx0 * ( (dA_dk * k - A) / (k * k) );
                const double dr1_dk_log = dr1_dk * k;
                const double dr1_dg = 0.0;

                // r2 derivatives
                const double dr2_dvx0 = 0.0;
                const double dr2_dvy0 = A / k;
                // dB/dk = -vy0/k^2 - 2*g/k^3
                const double dB_dk = -vy0 / (k * k) - 2.0 * g / (k * k * k);
                // dr2/dk = dB/dk * A + B * dA/dk + t * g / k^2
                const double dr2_dk = dB_dk * A + B * dA_dk + t * g / (k * k);
                const double dr2_dk_log = dr2_dk * k;
                const double dr2_dg = (1.0 / (k * k)) * A - t * (1.0 / k);

                // Place in row-major: row0 (r1): 4 entries, row1 (r2): next 4 entries
                J[0 * 4 + 0] = dr1_dvx0;
                J[0 * 4 + 1] = dr1_dvy0;
                J[0 * 4 + 2] = dr1_dk_log;
                J[0 * 4 + 3] = dr1_dg;

                J[1 * 4 + 0] = dr2_dvx0;
                J[1 * 4 + 1] = dr2_dvy0;
                J[1 * 4 + 2] = dr2_dk_log;
                J[1 * 4 + 3] = dr2_dg;
            }
            return true;
        }

        double t, xm, ym;
    };
    ceres::Solver::Summary solve(const std::vector<Measurement>& raw, Params& params, double fps)
    {
        double duration = raw.back().t - raw.front().t;
        ceres::Problem problem;
        params.vx0 = (raw.back().x - raw.front().x) / duration;
        params.vy0 = (raw.back().y - raw.front().y) / duration;
        params.k_log = std::log(1);    //to wit: k=1.0
        params.g = 100.0;
        for (const auto &m : raw) {
            // ceres::CostFunction* cf = BallCost::get_cost_function(m.t, m.x, m.y);
            // ceres::LossFunction* loss = new ceres::HuberLoss(2.0);
            // problem.AddResidualBlock(cf, nullptr, params.v);
            // Use analytic cost (faster than AutoDiff here)
            problem.AddResidualBlock(new BallCostAnalytic(m.t, m.x, m.y), nullptr, params.v);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 10;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        options.num_threads = std::max(1u, std::thread::hardware_concurrency());

        problem.SetParameterLowerBound(params.v, 3, 100);
        problem.SetParameterUpperBound(params.v, 3, 1000);
        problem.SetParameterLowerBound(params.v, 2, std::log(0.01));
        problem.SetParameterUpperBound(params.v, 2, std::log(1));
        options.num_linear_solver_threads = options.num_threads;
        options.use_nonmonotonic_steps = true;
        options.logging_type = ceres::LoggingType::SILENT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        return summary;
    }
}