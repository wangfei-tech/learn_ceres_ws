#pragma once

#include "eigen3/Eigen/Core"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_slam2d_api.h"

namespace g2o
{
    namespace slam2d
    {
        class G2O_SLAM2D_API VertexPointXY : public g2o::BaseVertex<2, Eigen::Vector2d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexPointXY() {}
            virtual void setToOriginImpl()
            {
                _estimate.setZero(); // 初始值
            }
            virtual void oplusImpl(const double *update)
            {
                // 更新点坐标
                _estimate[0] += update[0];
                _estimate[1] += update[1];
            }
            virtual bool read(std::istream &in);
            virtual bool write(std::ostream &out) const;
        };

    } // namespace slam2d

} // g2o