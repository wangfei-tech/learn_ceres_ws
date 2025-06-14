#pragma once
#include "g2o/core/base_binary_edge.h"
#include "g2o_slam2d_api.h"
#include "vertex_se2.h"

namespace g2o
{
    namespace slam2d
    {
        class G2O_SLAM2D_API EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                EdgeSE2();
                virtual void computeError()
                {
                    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
                    const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
                    SE2 delta =_inverseMeasurement * (v1->estimate().inverse() * v2->estimate());//计算观测误差当前估计的相对位姿” 与 “传感器观测到的相对位姿” 的差距
                    _error = delta.toVector();
                }
                void setMeasurement(const SE2 &m)
                {
                    _measurement = m;
                    _inverseMeasurement = m.inverse();
                }
                virtual bool read(std::istream &is);
                virtual bool write(std::ostream &os) const;

            protected:
                SE2 _inverseMeasurement;
        };
    }
}