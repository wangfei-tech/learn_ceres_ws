#ifndef CERES_EXAMPLES_POSE_GRAPH_2D_ANGLE_MANIFOLD_H_
#define CERES_EXAMPLES_POSE_GRAPH_2D_ANGLE_MANIFOLD_H_
#include "ceres/autodiff_manifold.h"
#include "normalize_angle.h"
namespace ceres::examples {
// Defines a manifold for updating the angle to be constrained in [-pi to pi).
// 定义一个流形，用于将角度更新限制在 [-pi 到 pi) 区间内。
class AngleManifold {
 public:
  template <typename T>
  bool Plus(const T* x_radians,
            const T* delta_radians,
            T* x_plus_delta_radians) const {
    *x_plus_delta_radians = NormalizeAngle(*x_radians + *delta_radians);
    return true;
  }
  template <typename T>
  bool Minus(const T* y_radians,
             const T* x_radians,
             T* y_minus_x_radians) const {
    *y_minus_x_radians =
        NormalizeAngle(*y_radians) - NormalizeAngle(*x_radians);
    return true;
  }
  static ceres::Manifold* Create() {
    return new ceres::AutoDiffManifold<AngleManifold, 1, 1>;
  }
};
}  // namespace ceres::examples
#endif  // CERES_EXAMPLES_POSE_GRAPH_2D_ANGLE_MANIFOLD_H_