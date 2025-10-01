// #include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "data.hpp"

namespace TASK03
{
    ceres::Solver::Summary solve(const std::vector<Measurement>& raw, Params& params, double fps);
}