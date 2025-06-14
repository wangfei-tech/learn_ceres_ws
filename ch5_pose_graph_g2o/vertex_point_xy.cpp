#include "vertex_point_xy.h"

namespace g2o
{
    namespace slam2d
    {
        bool VertexPointXY::read(std::istream &in) {
            Eigen::Vector2d v;
            in >> v[0] >> v[1];
            _estimate = v;
            return true;
        }
        bool VertexPointXY::write(std::ostream &out) const {
            out << _estimate[0] << " " << _estimate[1];
            return out.good();
        }

    }// namespace slam2d
}// namespace g2o