#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <absl/log/initialize.h>

// CostFunction 实现函数 r(x,y)=x⋅sin(y)−2 的解析导数。
class MultiVarCostFunction
    : public ceres::SizedCostFunction<1 /* number of residuals */,
                                      1 /* size of first parameter */,1>
{
public:
    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override
    {
        double x = parameters[0][0];
        double y = parameters[1][0];

        residuals[0] = x*std::sin(y) - 2;

        if (jacobians != nullptr )
        {
            if(jacobians[0]!=nullptr)//对x的导数
            {
                jacobians[0][0] = std::sin(y);
            }
            if(jacobians[1]!=nullptr)//对y的导数
            {
             jacobians[1][0] = x*std::cos(y);   
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
    ceres::CostFunction *cost_function = new MultiVarCostFunction;
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