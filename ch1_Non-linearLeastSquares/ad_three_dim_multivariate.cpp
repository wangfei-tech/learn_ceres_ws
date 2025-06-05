#include <cmath>
#include <ceres/ceres.h>
#include <absl/log/initialize.h>

class ThreeDimCostFunction : public ceres::SizedCostFunction<3, 1, 1, 1>
{
    bool Evaluate(double const *const *parametres,
                  double *residuals,
                  double **jacobians) const override
    {
        double x = parametres[0][0];
        double y = parametres[1][0];
        double z = parametres[2][0];
        residuals[0] = x + y + z - 6; // 第一个残差 r1 = x + y + z - 6
        residuals[1] = x * y + z - 5; // 第二个残差 r2 = x * y - 5
        residuals[2] = x * y * z - 4; // 第三个残差 r3 = x * y * z - 4
        if (jacobians)
        {
            if (jacobians[0] != nullptr) // 对 x 的导数
            {
                jacobians[0][0] = 1.0; // r1 对 x 的导数
                jacobians[0][1] = y;   // r2 对 x 的导数
                jacobians[0][2] = y * z; // r3 对 x 的导数
            }
            if (jacobians[1] != nullptr) // 对 y 的导数
            {
                jacobians[1][0] = 1.0; // r1 对 y 的导数
                jacobians[1][1] = x;   // r2 对 y 的导数
                jacobians[1][2] = x * z; // r3 对 y 的导数
            }
            if (jacobians[2] != nullptr) // 对 z 的导数
            {
                jacobians[2][0] = 1.0; // r1 对 z 的导数
                jacobians[2][1] = 1.0; // r2 对 z 的导数
                jacobians[2][2] = x * y; // r3 对 z 的导数
            }
        }
        return true;
    }
};
int main(int argc,char **argv)
{
    absl::InitializeLog();
    double x = 1.0;
    double y = 1.0;
    double z = 1.0;
    const double initial_x = x;
    const double initial_y = y;
    const double initial_z = z;

    ceres::Problem problem;
    ceres::CostFunction *cost_function = new ThreeDimCostFunction;
    problem.AddResidualBlock(cost_function, nullptr, &x, &y, &z);
    // Run the solver!
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_x << " -> " << y << "\n";
    std::cout << "z : " << initial_x << " -> " << z << "\n";

}