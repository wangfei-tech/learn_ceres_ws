#include <vector>
#include <ceres/ceres.h>
#include <absl/log/initialize.h>

// CostFunction 实现函数 f(x) = 10 - x 的解析导数。
class QuadraticCostFunction
    : public ceres::SizedCostFunction<1 /* number of residuals */,
                                      1 /* size of first parameter */>
{
public:
    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override
    {
        double x = parameters[0][0];
        // f(x) = 10 - x.
        residuals[0] = 10 - x;

        if (jacobians != nullptr && jacobians[0] != nullptr)
        {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

int main(int argc ,char **argv)
{
    absl::InitializeLog();
    double x = 0.5;
    const double initial_x = x;
    ceres::Problem problem;
    // 使用解析导数计算雅可比矩阵，最小化 0.5 * (10 - x)^2
    ceres::CostFunction *cost_function = new QuadraticCostFunction;
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