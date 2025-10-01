#include "solver.hpp"

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
    ceres::Solver::Summary solve(const std::vector<Measurement>& raw, Params& params, double fps)
    {
        double duration = raw.back().t - raw.front().t;
        ceres::Problem problem;
        params.vx0 = (raw.back().x - raw.front().x) / duration;
        params.vy0 = (raw.back().y - raw.front().y) / duration;
        params.k_log = std::log(1);    //to wit: k=1.0
        params.g = 100.0;
        for (const auto &m : raw) {
            ceres::CostFunction* cf = BallCost::get_cost_function(m.t, m.x, m.y);
            // ceres::LossFunction* loss = new ceres::HuberLoss(2.0);
            problem.AddResidualBlock(cf, nullptr, params.v);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 10;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        problem.SetParameterLowerBound(params.v, 3, 100);
        problem.SetParameterUpperBound(params.v, 3, 1000);
        problem.SetParameterLowerBound(params.v, 2, std::log(0.01));
        problem.SetParameterUpperBound(params.v, 2, std::log(1));

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        return summary;
    }
}