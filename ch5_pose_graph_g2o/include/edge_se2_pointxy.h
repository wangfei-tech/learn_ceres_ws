#pragma once

#include "g2o/core/base_binary_edge.h"
#include "g2o_slam2d_api.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "parameter_se2_offset.h"
#include "g2o/core/parameter_container.h"

using namespace g2o::slam2d;
namespace g2o
{
    namespace slam2d
    {

        // class g2o::slam2d::ParameterSE2Offset;
        // class g2o::slam2d::CacheSE2Offset;

        class G2O_SLAM2D_API EdgeSE2PointXY
            : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            EdgeSE2PointXY();

            void computeError();

            virtual bool read(std::istream &is);
            virtual bool write(std::ostream &os) const;

        protected:
            g2o::slam2d::ParameterSE2Offset *_sensorOffset;
            g2o::slam2d::CacheSE2Offset *_sensorCache;

            virtual bool resolveCaches();
        };

    } // namespace slam2d
} // namespace g2o