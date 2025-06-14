#include "./include/edge_se2.h"

using namespace Eigen;

namespace g2o
{
    namespace slam2d
    {
        EdgeSE2::EdgeSE2() : BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>() {}

        bool EdgeSE2::read(std::istream &is)
        {
            Vector3d p;
            is >> p[0] >> p[1] >> p[2];
            _measurement.fromVector(p);
            _inverseMeasurement = _measurement.inverse();
            for (int i = 0; i < 3; ++i)
                for (int j = i; j < 3; ++j)
                {
                    is >> information()(i, j);
                    if (i != j)
                        information()(j, i) = information()(i, j);
                }
            return true;
            return true;
        }

        bool EdgeSE2::write(std::ostream &os) const
        {
            Vector3d p = _measurement.toVector(); // LOG
            os << p[0] << " " << p[1] << " " << p[2] << " ";
            for (int i = 0; i < 3; ++i)
                for (int j = i; j < 3; ++j)
                {
                    os << information()(i, j) << " ";
                }
            return os.good();
        }

    } // namespace slam2d

} // namespace g2o