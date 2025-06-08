#ifndef CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
#define CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
#include <cmath>
#include "ceres/ceres.h"

namespace ceres::examples {
// 将角度归一化到 [-pi, pi) 区间内。
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // 使用 ceres::floor 因为它专门用于 double 和 Jet 类型。
  // ceres::floor 是为了处理 Jet<T, N> 类型而提供的重载函数，用于兼容 Ceres 的自动微分机制
  // Jet<T, N> 是 Ceres Solver 中用于自动微分（Automatic Differentiation）的核心数据结构，它表示一个“值 + 导数”的组合
  // T 是一个模板参数，表示数值类型（如 double 或 float），而 N 是 Jet 的阶数，表示导数的数量。
  T two_pi(2.0 * constants::pi);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(constants::pi)) / two_pi);
}
}  // namespace ceres::examples
#endif  // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
