#ifndef EXAMPLES_CERES_READ_G2O_H_
#define EXAMPLES_CERES_READ_G2O_H_
#include <fstream>
#include <functional>
#include <istream>
#include <map>
#include <string>
#include <vector>
#include "absl/log/check.h"
#include "absl/log/log.h"
namespace ceres::examples
{
    // 读取位姿并将其插入到位姿图中，如果位姿图中已经存在该位姿，则返回false。
    template <typename Pose, typename Allocator>
    bool ReadVertex(std::ifstream *infile,
                    std::map<int, Pose, std::less<int>, Allocator> *poses)
    {
        int id;
        Pose pose;
        *infile >> id >> pose;
        // Ensure we don't have duplicate poses.
        if (poses->find(id) != poses->end())
        {
            LOG(ERROR) << "Duplicate vertex with ID: " << id;
            return false;
        }
        (*poses)[id] = pose;
        return true;
    }
    // 读取位姿图中两个顶点之间的约束
    template <typename Constraint, typename Allocator>
    void ReadConstraint(std::ifstream *infile,
                        std::vector<Constraint, Allocator> *constraints)
    {
        Constraint constraint;
        *infile >> constraint;
        constraints->push_back(constraint);
    }
    // 读取一个以 g2o 文件名格式的文件，该文件描述了一个位姿图
    // 问题。g2o 格式包含两个条目：顶点和约束。
    //
    // 在 2D 中，一个顶点被定义为：
    //
    // VERTEX_SE2 ID x_meters y_meters yaw_radians
    //
    // 约束被定义为：
    //
    // EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
    //
    // 其中 I_ij 是测量信息矩阵的第 (i, j) 个条目。
    //
    //
    // 在 3D 中，一个顶点被定义为：
    //
    // VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
    //
    // 其中四元数是汉密尔顿 Hamilton形式。
    // 
    // 约束被定义为：
    //
    // EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
    //
    // where I_ij is the (i, j)-th entry of the information matrix for the
    // measurement. 仅仅保存上三角的测量 测量顺序是先计算位置的增量再计算方向的增量
    template <typename Pose,
              typename Constraint,
              typename MapAllocator,
              typename VectorAllocator>
    bool ReadG2oFile(const std::string &filename,
                     std::map<int, Pose, std::less<int>, MapAllocator> *poses,
                     std::vector<Constraint, VectorAllocator> *constraints)
    {
        CHECK(poses != nullptr);
        CHECK(constraints != nullptr);
        poses->clear();
        constraints->clear();
        std::ifstream infile(filename.c_str());
        if (!infile)
        {
            return false;
        }
        std::string data_type;
        while (infile.good())
        {
            // Read whether the type is a node or a constraint.
            infile >> data_type;
            if (data_type == Pose::name())
            {
                if (!ReadVertex(&infile, poses))
                {
                    return false;
                }
            }
            else if (data_type == Constraint::name())
            {
                ReadConstraint(&infile, constraints);
            }
            else
            {
                LOG(ERROR) << "Unknown data type: " << data_type;
                return false;
            }
            // Clear any trailing whitespace from the line.
            infile >> std::ws;
        }
        return true;
    }
} // namespace ceres::examples
#endif // EXAMPLES_CERES_READ_G2O_H_
