#include <iostream>
#include <memory>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <eigen3/Eigen/Core>

using namespace g2o;
using namespace std;

// 1维变量顶点，继承BaseVertex<维度, 数据类型>
class VertexX : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexX() = default;

    virtual void setToOriginImpl() override
    {
        _estimate = 0.0;
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate += *update;
    }

    // 新增的必须实现的函数
    virtual bool read(std::istream &is) override { return true; }
    virtual bool write(std::ostream &os) const override { return true; }

    // 新增的堆栈操作函数 - 对于简单顶点可以空实现
    virtual void push() override {}
    virtual void pop() override {}
    virtual void discardTop() override {}
    virtual int stackSize() const override { return 0; }
};

// 代价函数边，继承BaseUnaryEdge<维度, 测量值类型, 顶点类型>
class EdgeResidual : public g2o::BaseUnaryEdge<1, double, VertexX>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeResidual(double measurement) : _measurement(measurement) {}

    virtual void computeError() override
    {
        const VertexX *v = static_cast<const VertexX *>(_vertices[0]);
        _error[0] =  v->estimate() - _measurement ; // 计算误差
    }

    // 关键新增：提供解析雅可比矩阵
    virtual void linearizeOplus() override {
        _jacobianOplusXi[0] = 1.0; // 对于线性模型，导数为1
    }
    // 新增的必须实现的函数
    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }

private:
    double _measurement;
};

int main()
{

    // 1. 创建优化器
    SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // 2. 设置求解器
    using BlockSolverType = BlockSolver<BlockSolverTraits<1, 1>>;
    using LinearSolverType = LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto solver = new OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver);

    // 3. 添加顶点
    auto *v = new VertexX();
    v->setId(0);
    v->setEstimate(0.0);
    optimizer.addVertex(v);

    // 4. 添加边
    auto *edge = new EdgeResidual(10.0);
    edge->setId(0);
    edge->setVertex(0, v);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(edge);

    // 5. 优化前检查
    if (!optimizer.initializeOptimization())
    {
        std::cerr << "Error initializing optimization!" << std::endl;
        return -1;
    }

    // 6. 执行优化
    int iterations = 10;
    optimizer.optimize(iterations);

    // 7. 输出结果
    std::cout << "\nOptimization results:" << std::endl;
    std::cout << "Number of vertices: " << optimizer.vertices().size() << std::endl;
    std::cout << "Number of edges: " << optimizer.edges().size() << std::endl;
    std::cout << "Optimized x: " << v->estimate() << std::endl;
    std::cout << "Expected optimal value: 10.0" << std::endl;

    // 8. 清理内存
    optimizer.clear();

    return 0;
}