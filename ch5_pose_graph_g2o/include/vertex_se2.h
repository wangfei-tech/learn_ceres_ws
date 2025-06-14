
/// 顶点的定义以及读写操作
#pragma once

#include "g2o/core/base_vertex.h"
#include "g2o_slam2d_api.h"
#include "g2o/core/hyper_graph_action.h"
#include "eigen3/Eigen/Core"
#include "se2.h"

namespace g2o
{
    namespace slam2d
    {
        /*声明2D pose 位姿(x,y,theta)*/
        /**跨平台写法 class G2O_SLAM2D_API VertexSE2 确保 SE2 类可以被正确地
         * 导出到共享库中，或者从共享库中导入出来，从而在不同平台/不同项目中安全使用。
         * 是大型项目中 模块化编程（封装成库） 的标准做法。*/
        class G2O_SLAM2D_API VertexSE2 : public g2o::BaseVertex<3, SE2>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexSE2() {}

            virtual void setToOriginImpl() 
            {
                _estimate = SE2(); // 初始值
            }
            virtual void oplusImpl(const double *update) 
            {
                // 更新位姿
                SE2 up(update[0], update[1], update[2]);
                _estimate *= up; // 位姿乘法
            }
            virtual bool read(std::istream &in) ;
            virtual bool write(std::ostream &out) const ;
        };

    } // namespace slam2d
} // namespace g2o