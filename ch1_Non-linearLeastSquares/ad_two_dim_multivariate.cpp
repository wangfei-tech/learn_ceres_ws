#include <cmath>
#include <ceres/ceres.h>
#include <absl/log/initialize.h>

// 残差向量维度为 2，参数 x 和 y 各是 1 维
class TwoDimResidual : public ceres::SizedCostFunction<2 /* number of residuals */,
                                                       1 /* size of first parameter */, 1 /* size of second parameter */>
{
public:
    bool Evaluate(double const *const *parameter,
                  double *residuals,
                  double **jacobians) const override
    {
        double x = parameter[0][0];
        double y = parameter[1][0];
        residuals[0] = x + y - 3; // 第一个残差 r1 = x + y - 3
        residuals[1] = x * y - 2; // 第二个残差 r2 = x * y - 2
        if (jacobians)
        {
            if (jacobians[0] != nullptr)// 对 x 的导数
            {
                jacobians[0][0] = 1.0; 
                jacobians[0][1] = y; 
            }
            if (jacobians[1] != nullptr)// 对 y 的导数
            {
                jacobians[1][0] = 1.0; 
                jacobians[1][1] = x; 
            }
        }

        return true;
    }
};

int main(int argc ,char **argv)
{
    absl::InitializeLog();
    double x = 1.0;
    double y= 1.0;
    const double initial_x = x;
    const double initial_y = y;
    ceres::Problem problem;
    ceres::CostFunction *cost_function = new TwoDimResidual;
    problem.AddResidualBlock(cost_function, nullptr, &x, &y);
    // Run the solver!
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;   
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_y << " -> " << y << "\n";
    return 0;
}

