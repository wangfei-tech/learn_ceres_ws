/**在实际 SLAM 中，经常存在传感器的坐标系与机器人基座坐标系不一致的情况：
    传感器安在机器人边缘，比如：
    激光雷达并不在 robot_base_link 中心，而是偏移了一段距离
    或者：存在外参（Extrinsic）变换
    这时候，我们就需要一个 offset 参数
    把“机器人位姿”变换到“传感器位姿”
    或者反过来，从传感器位姿回推机器人位姿
*/

#ifndef G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H
#define G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H

#include "g2o/core/cache.h"
#include "g2o_slam2d_api.h"
#include "se2.h"

namespace g2o
{
    namespace slam2d
    {

        class G2O_SLAM2D_API ParameterSE2Offset : public Parameter
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            ParameterSE2Offset();

            void setOffset(const SE2 &offset = SE2());

            const SE2 &offset() const { return _offset; }
            const SE2 &inverseOffset() const { return _inverseOffset; }

            virtual bool read(std::istream &is);
            virtual bool write(std::ostream &os) const;

        protected:
            SE2 _offset;
            SE2 _inverseOffset;
        };// end class ParameterSE2Offset

        class G2O_SLAM2D_API CacheSE2Offset : public Cache
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            const SE2 &w2n() const { return _w2n; }
            const SE2 &n2w() const { return _n2w; }

        protected:
            virtual void updateImpl();
            virtual bool resolveDependencies();

            ParameterSE2Offset *_offsetParam;
            SE2 _w2n, _n2w;
        };// end class CacheSE2Offset

    } // end namespace tutorial
} // namespace g2o

#endif