#include <iostream>
#include <memory>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <eigen3/Eigen/Core>

using namespace g2o;
using namespace std;

// 定义顶点：待优化的参数： a,b,c
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override
    {
        _estimate << 0.0, 0.0, 0.0; // 初始值
    }
    virtual void oplusImpl(const double *update) override
    {
        _estimate += Eigen::Vector3d(update); // 参数更新
    }
    virtual bool read(std::istream &in) override { return false; }
    virtual bool write(std::ostream &out) const override { return false; }
    virtual void push() override {}
    virtual void pop() override {}
    virtual void discardTop() override {}
    virtual int stackSize() const override { return 0; }
};

// 定义边进行误差计算
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x) : _x(x) {}

    // 计算误差
    virtual void computeError() override
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error[0] = _measurement - abc[0] * _x * _x - abc[1] * _x - abc[2]; // e = y - (ax² + bx + c)
    }

    virtual void linearizeOplus() override
    {
        _jacobianOplusXi[0] = -_x * _x; // ∂e/∂a = -x²
        _jacobianOplusXi[1] = -_x;      // ∂e/∂b = -x
        _jacobianOplusXi[2] = -1;       // ∂e/∂c = -1
    }
    virtual bool read(std::istream &in) override { return false; }
    virtual bool write(std::ostream &out) const override { return false; }

private:
    double _x;
};

int main()
{
    // 生成带噪声的观测数据 y = 1.0x² + 2.0x + 3.0 + 噪声
    std::vector<double> x_data, y_data;
    for (double x = 0; x <= 4; x += 0.1)
    {
        double y = 1.0 * x * x + 2.0 * x + 3.0 + 0.2 * (rand() % 100 / 100.0 - 0.5);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    // 创建优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true); // 设置输出详细信息

    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolverX>(
        std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));
    optimizer.setAlgorithm(solver);

    // 添加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setId(0);
    v->setEstimate(Eigen::Vector3d(0, 0, 0)); // 设置初始值

    optimizer.addVertex(v);
    // 添加边(观测数据)

    for (int i = 0; i < x_data.size(); i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);                                         // 设置顶点
        edge->setMeasurement(y_data[i]);                               // 设置观测值
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // 信息矩阵
        //Huber Loss（胡伯损失）是一种常用于鲁棒回归与SLAM优化问题中的损失函数，
        //它结合了 L2（平方）损失和 L1（绝对值）损失的优点，在保留小残差精度的同时抑制大残差的影响，适合包含少量异常值的数据优化问题。
        //为什么 SLAM 优化常用 Huber Loss？
        //纯 L2 损失对异常值非常敏感，容易导致优化结果被“拉偏”。
        //纯 L1 虽然鲁棒但不可导/不连续，难以用于高效的二阶优化方法（如高斯-牛顿）。
        //Huber Loss 在小残差时保留 L2 的精确性，大残差时用 L1 增强鲁棒性，并且保持连续可导，非常适合图优化（g2o、Ceres等）
        auto* kernel = new g2o::RobustKernelHuber();
        kernel->setDelta(5.0);  // delta 是鲁棒性调节参数，越大越“温和” 
        edge->setRobustKernel(kernel);
        optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization(); // 初始化优化器
    optimizer.optimize(10);             // 执行优化，迭代10次
    // 输出优化结果
    Eigen::Vector3d abc = v->estimate();
    std::cout << "Optimized parameters: a = " << abc[0]
              << ", b = " << abc[1]
              << ", c = " << abc[2] << std::endl;
    return 0;
}