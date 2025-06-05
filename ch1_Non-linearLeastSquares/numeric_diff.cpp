// 使用数值微分计算雅可比矩阵，最小化 0.5 (10 - x)^2

#include <ceres/ceres.h>
#include "absl/log/initialize.h"

// 定义残差函数
struct CostFunctor
{
    bool operator()(const double *const x, double *residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int agrc, char **argv)
{
    absl::InitializeLog();
    double x = 0.5;
    const double initial_x = x;
    // 声明问题解析器
    ceres::Problem problem;

    // 使用数值积分进行计算
    ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>(
        new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // Run the solver!
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    return 0;
}
