#ifndef CERES_EXAMPLES_POSE_GRAPH_2D_TYPES_H_
#define CERES_EXAMPLES_POSE_GRAPH_2D_TYPES_H_
#include <fstream>
#include "eigen3/Eigen/Core"
#include "normalize_angle.h"
namespace ceres::examples {
// 位姿图中每个顶点的状态。
struct Pose2d {
  double x;
  double y;
  double yaw_radians;
  // 在 g2o 文件格式中的数据类型名称。
  static std::string name() { return "VERTEX_SE2"; }
};
inline std::istream& operator>>(std::istream& input, Pose2d& pose) {
  input >> pose.x >> pose.y >> pose.yaw_radians;
  // NormalizeAngle函数将角度归一化到[-pi, pi)区间内。
  pose.yaw_radians = NormalizeAngle(pose.yaw_radians);
  return input;
}
// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
//  位姿图中两个顶点之间的约束。约束是从顶点 id_begin 到顶点 id_end 的变换。

struct Constraint2d {
  int id_begin;
  int id_end;
  double x;
  double y;
  double yaw_radians;
  // 信息矩阵，表示约束的精度。
  Eigen::Matrix3d information;
  // The name of the data type in the g2o file format.
  // 在 g2o 文件格式中的数据类型名称。
  static std::string name() { return "EDGE_SE2"; }
};
inline std::istream& operator>>(std::istream& input, Constraint2d& constraint) {
  input >> constraint.id_begin >> constraint.id_end >> constraint.x >>
      constraint.y >> constraint.yaw_radians >> constraint.information(0, 0) >>
      constraint.information(0, 1) >> constraint.information(0, 2) >>
      constraint.information(1, 1) >> constraint.information(1, 2) >>
      constraint.information(2, 2);
  // 设置信息矩阵的下三角部分。
  constraint.information(1, 0) = constraint.information(0, 1);
  constraint.information(2, 0) = constraint.information(0, 2);
  constraint.information(2, 1) = constraint.information(1, 2);
  // 将角度归一化到 [-pi, pi) 区间内。
  constraint.yaw_radians = NormalizeAngle(constraint.yaw_radians);
  return input;
}
}  // namespace ceres::examples
#endif  // CERES_EXAMPLES_POSE_GRAPH_2D_TYPES_H_