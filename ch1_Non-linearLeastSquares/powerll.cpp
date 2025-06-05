#include <iostream>
#include <string>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "ceres/ceres.h"
struct F1
{
    template <typename T>
    bool operator()(const T *const x1, const T *const x2, T *residual) const
    {
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};
struct F2
{
    template <typename T>
    bool operator()(const T *const x3, const T *const x4, T *residual) const
    {
        residual[0] = std::sqrt(5.0) * (x3[0] - x4[0]);
        return true;
    }
};
struct F3
{
    template <typename T>
    bool operator()(const T *const x2, const T *const x3, T *residual) const
    {
        residual[0] = (x2[0] - 2.0 * x3[0])*(x2[0] - 2.0 * x3[0]);
        return true;
    }
};
struct F4
{
    template <typename T>
    bool operator()(const T *const x1, const T *const x4, T *residual) const
    {
        residual[0] = std::sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};
/// Define the flags for the parameters
//定义命令行参数 --minimizer，默认值为 trust_region，并附带说明文字。它允许用户在启动程序时通过命令行指定优化器类型。
ABSL_FLAG(std::string,
          minimizer,
          "trust_region",
          "Minimizer type to use, choices are: line_search & trust_region");
int main(int argc, char **argv)
{
    absl::InitializeLog();
    absl::ParseCommandLine(argc, argv);
    double x1 = 3.0;
    double x2 = -1.0;
    double x3 = 0.0;
    double x4 = 1.0;
    ceres::Problem problem;
    // 使用自动微分包装器向问题中添加残差项，以自动获取导数。参数 x1 到 x4 均已进行修改。
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, &x1, &x2);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x3, &x4);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x2, &x3);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, &x1, &x4);
    ceres::Solver::Options options;
    LOG_IF(FATAL,
           !ceres::StringToMinimizerType(absl::GetFlag(FLAGS_minimizer),
                                         &options.minimizer_type))        //options.minimizer_type 是用来设置求解器使用哪种**非线性最小二乘优化策略
        << "Invalid minimizer: " << absl::GetFlag(FLAGS_minimizer)
        << ", valid options are: trust_region and line_search.";
    options.max_num_iterations = 100; // 最大迭代次数
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    std::cout << "Initial x1 = " << x1
              << ", x2 = " << x2
              << ", x3 = " << x3
              << ", x4 = " << x4
              << "\n";
    // Run the solver!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    // clang-format off
    std::cout << "Final x1 = " << x1
            << ", x2 = " << x2
            << ", x3 = " << x3
            << ", x4 = " << x4
            << "\n";
    return 0;
}
