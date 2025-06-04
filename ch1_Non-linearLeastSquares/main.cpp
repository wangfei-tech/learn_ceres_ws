#include "ceres/ceres.h"
#include "absl/log/initialize.h"

// 这是一个简单的例子，展示了如何使用Ceres Solver来求解非线性最小二乘问题。

// 我们将定义一个代价函数，并使用Ceres来最小化它。
// 一个模板化的代价函数，实现了残差 r = 10 -
// x。方法operator()是模板化的，因此我们可以使用它的
// 自动微分包装器来生成它的
// 导数。
struct CostFunctor
{
    template <typename T>
    bool operator()(const T *const x, T *residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char **argv)
{
    absl::InitializeLog(); // 初始化日志系统
    // 先定义一个变量x，初始值为0.5
    double x = 0.5;
    const double initial_x = x;
    // 创建一个Ceres问题对象
    ceres::Problem problem;

    // 设置唯一的成本函数（也称为残差）。这将使用自微分来获取导数（雅可比矩阵）。
    // 这里我们使用AutoDiffCostFunction来自动计算导数。
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor()); // 1个残差，1个参数
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // Run the solver!
    ceres::Solver::Options options;// 求解器选项
    // 设置求解器选项
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true; // 输出求解过程到标准输出
    ceres::Solver::Summary summary;// 求解器总结信息
    ceres::Solve(options, &problem, &summary);// 执行求解
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    return 0;
}
