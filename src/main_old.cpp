//
//  main.cpp
//  Created by Leo Xia on 2025/10/1
//  Created with VSCode on Ubuntu 22.04
//

#include <iostream>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

// 残差函数对象：10 - x，想让最终结果趋近于 10
struct CostFunctor {
  template <typename T>                 // 模板化，供 Ceres 的自动微分使用
  bool operator()(const T* const x,     // 优化变量（1 维）
                  T* residual) const { // 输出的残差
    residual[0] = T(10.0) - x[0];       // 残差 = 10 - x
    return true;                        // 返回 true 表示计算成功
  }
};

int main(int argc, char** argv) {
//   google::InitGoogleLogging(argv[0]);   // 初始化 glog，打印调试信息

  double x = 0.0;                       // 待优化变量的初值，从 0 开始

  ceres::Problem problem;               // 构造一个优化问题（后面会 AddResidualBlock）
    // 把残差块挂到问题里：1 个残差维度，1 个参数块维度，使用自动微分
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(
          new CostFunctor),
      nullptr,   // 无核函数
      &x);       // 待优化变量

  // 配置并求解
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << x << " -> 10\n";
  return 0;
}