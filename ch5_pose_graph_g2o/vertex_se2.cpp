#include "./include/vertex_se2.h"

namespace g2o
{
    namespace slam2d
    {
        bool VertexSE2::read(std::istream &in)
        {
            Eigen::Vector3d v;
            in >> v[0] >> v[1] >> v[2];
            _estimate.fromVector(v);
            return true;
        }

        bool VertexSE2::write(std::ostream &out) const
        {
            Eigen::Vector3d v = _estimate.toVector();
            out << v[0] << " " << v[1] << " " << v[2];
            return out.good();
        }

    }// namespace slam2d
    
} // namespace g2o

