#include "./include/parameter_se2_offset.h"

#include "./include/vertex_se2.h"

namespace g2o
{
    namespace slam2d
    {

        ParameterSE2Offset::ParameterSE2Offset() {}

        void ParameterSE2Offset::setOffset(const SE2 &offset)
        {
            _offset = offset;
            _inverseOffset = offset.inverse();
        }

        bool ParameterSE2Offset::read(std::istream &is)
        {
            double x, y, th;
            is >> x >> y >> th;
            setOffset(SE2(x, y, th));
            return true;
        }

        bool ParameterSE2Offset::write(std::ostream &os) const
        {
            os << _offset.translation().x() << " " << _offset.translation().y() << " "
               << _offset.rotation().angle();
            return os.good();
        }

        void CacheSE2Offset::updateImpl()
        {
            const VertexSE2 *v = static_cast<const VertexSE2 *>(vertex());
            _n2w = v->estimate() * _offsetParam->offset();
            _w2n = _n2w.inverse();
        }

        bool CacheSE2Offset::resolveDependencies()
        {
            _offsetParam = dynamic_cast<ParameterSE2Offset *>(_parameters[0]);
            return _offsetParam != 0;
        }
        

    } // end namespace tutorial
} // namespace g2o