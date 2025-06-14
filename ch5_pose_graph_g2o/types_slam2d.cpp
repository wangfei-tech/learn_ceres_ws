#include "./include/types_slam2d.h"

#include <iostream>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/cache.h"  // 确保包含 cache 系统头文件

namespace g2o {
namespace slam2d {

G2O_REGISTER_TYPE_GROUP(tutorial_slam2d);

G2O_REGISTER_TYPE_NAME("TUTORIAL_VERTEX_SE2", VertexSE2);
G2O_REGISTER_TYPE_NAME("TUTORIAL_VERTEX_POINT_XY", VertexPointXY);

G2O_REGISTER_TYPE_NAME("TUTORIAL_PARAMS_SE2_OFFSET", ParameterSE2Offset);

G2O_REGISTER_TYPE_NAME("TUTORIAL_CACHE_SE2_OFFSET", CacheSE2Offset);



G2O_REGISTER_TYPE_NAME("TUTORIAL_EDGE_SE2", EdgeSE2);

G2O_REGISTER_TYPE_NAME("TUTORIAL_EDGE_SE2_POINT_XY", EdgeSE2PointXY);

}  // namespace slam2d
}  // namespace g2o
int main()
{
    return 0;
}