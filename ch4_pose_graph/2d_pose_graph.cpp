#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "include/angle_manifold.h"
#include "ceres/ceres.h"
#include "include/read_g2o.h"
#include "types.h"
#include "pose_graph_2d_error_term.h"

ABSL_FLAG(std::string,
          input,
          "",
          "The pose graph definition filename in g2o format.");

namespace ceres::examples
{
    namespace
    {
        // Constructs the nonlinear least squares optimization problem from the pose
        // graph constraints.
        // 构建非线性最小二乘优化问题，来自位姿图约束。
        void BuildOptimizationProblem(const std::vector<Constraint2d> &constraints,
                                      std::map<int, Pose2d> *poses,
                                      ceres::Problem *problem)
        {
            CHECK(poses != nullptr);
            CHECK(problem != nullptr);
            if (constraints.empty())
            {
                LOG(INFO) << "No constraints, no problem to optimize.";
                return;
            }
            // 损失函数
            ceres::LossFunction *loss_function = nullptr;
            // 流形空间
            ceres::Manifold *angle_manifold = AngleManifold::Create();
            // 遍历所有的约束
            for (auto constraint : constraints)
            {
                auto pose_begin_iter = poses->find(constraint.id_begin);
                CHECK(pose_begin_iter != poses->end())
                    << "Pose with ID " << constraint.id_begin
                    << " not found in the pose graph.";
                auto pose_end_iter = poses->find(constraint.id_end);
                CHECK(pose_end_iter != poses->end())
                    << "Pose with ID: " << constraint.id_end << " not found.";

                // 将一个对称正定的信息矩阵做 Cholesky 分解，获取其下三角“平方根”矩阵，以用于优化中误差项的加权。
                const Eigen::Matrix3d sqrt_information =
                    constraint.information.llt().matrixL(); // 表示对正定对称矩阵 information 进行 Cholesky（LLT）分解
                //
                ceres::CostFunction *cost_function = ceres::examples::PoseGraph2dErrorTerm::Create(
                    constraint.x, constraint.y, constraint.yaw_radians,
                    sqrt_information);
                // 添加残差块到问题中。
                problem->AddResidualBlock(cost_function,
                                          loss_function,
                                          &pose_begin_iter->second.x,
                                          &pose_begin_iter->second.y,
                                          &pose_begin_iter->second.yaw_radians,
                                          &pose_end_iter->second.x,
                                          &pose_end_iter->second.y,
                                          &pose_end_iter->second.yaw_radians);
                // 设置流形空间，确保角度在 [-pi, pi) 区间内。
                problem->SetManifold(&pose_begin_iter->second.yaw_radians, angle_manifold);
                problem->SetManifold(&pose_end_iter->second.yaw_radians, angle_manifold);
            }
            auto pose_start_iter = poses->begin();
            CHECK(pose_start_iter != poses->end())
                << "There is no pose in the pose graph.";
            // 位姿图优化中存在 三个自由度的冗余（gauge freedom），
            // 这会导致优化器结果不唯一或发散，
            // 最好通过将某个节点（如第一个）设为常量来固定坐标系，消除这种自由度。
            /*在位姿图（Pose Graph）中，我们优化的是相对约束：
            例如：节点 A 和节点 B 之间的相对位姿。
            如果你把整个图整体平移、旋转，所有的相对位姿都不变，所以代价函数也不变。
            所以优化器会发现 “怎么动都一样”，出现不收敛或漂移的情况。
            */
            problem->SetParameterBlockConstant(&pose_start_iter->second.x);
            problem->SetParameterBlockConstant(&pose_start_iter->second.y);
            problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
        }

        bool SolveOptimizationProblem(ceres::Problem *problem)
        {
            CHECK(problem != nullptr);
            // 设置求解器选项
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 100;
            // 求解问题
            ceres::Solver::Summary summary;
            ceres::Solve(options, problem, &summary);
            LOG(INFO) << summary.FullReport();
            return summary.IsSolutionUsable();
        }
        bool OutputPoses(const std::string &filename,
                         const std::map<int, Pose2d> &poses)
        {
            std::fstream output_file(filename);
            output_file.open(filename.c_str(), std::istream::out);
            if (!output_file)
            {
                std::cerr << "Error opening the file: " << filename << '\n';
                return false;
            }
            for (const auto &pair : poses)
            {
                output_file << pair.first << " " << pair.second.x << " " << pair.second.y << ' '
                            << pair.second.yaw_radians << '\n';
            }
            return true;
        }
    } // namespace
} // namespace ceres::examples

int main(int argc, char **argv)
{
    absl::InitializeLog();
    absl::ParseCommandLine(argc, argv);
    if (absl::GetFlag(FLAGS_input).empty())
    {
        LOG(ERROR) << "Please specify the input file with --input flag.";
        return 1;
    }
    std::map<int, ceres::examples::Pose2d> poses;
    std::vector<ceres::examples::Constraint2d> constraints;
    CHECK(ceres::examples::ReadG2oFile(absl::GetFlag(FLAGS_input), &poses, &constraints))
        << "Failed to read the pose graph from file: " << absl::GetFlag(FLAGS_input);
    std::cout << "Number of poses: " << poses.size() << '\n';
    std::cout << "Number of constraints: " << constraints.size() << '\n';
    CHECK(ceres::examples::OutputPoses("../data/poses_original.txt", poses))
        << "Error outputting to poses_original.txt";

    ceres::Problem problem;
    ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
    CHECK(ceres::examples::SolveOptimizationProblem(&problem))
        << "Failed to solve the optimization problem.";
    CHECK(ceres::examples::OutputPoses("../data/poses_optimized.txt", poses))
        << "Error outputting to poses_original.txt";
    return 0;
}