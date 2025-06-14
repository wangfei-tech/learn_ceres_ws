cmake_minimum_required(VERSION 3.10)
project(my_g2o_example)
set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0072 NEW)

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${G2O_INCLUDE_DIRS}
)

add_library(slam2d_library SHARED  
    include/se2.h    
    include/vertex_point_xy.h
    include/edge_se2_pointxy.h   
    include/edge_se2.h 
    include/vertex_se2.h
    include/parameter_se2_offset.h
    include/types_slam2d.h
    include/simulator.h
    include/g2o_slam2d_api.h
    vertex_se2.cpp
    simulator.cpp
    types_slam2d.cpp
    vertex_point_xy.cpp 
    parameter_se2_offset.cpp
    edge_se2.cpp  
    edge_se2_pointxy.cpp  
)

set_target_properties(slam2d_library PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME ${LIB_PREFIX}slam2d_library
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION}
)

target_link_libraries(slam2d_library
  g2o::core
  g2o::solver_eigen
  Eigen3::Eigen
)

add_executable(tutorial_slam2d
  tutorial_slam2d.cpp
)

set_target_properties(tutorial_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d)

target_link_libraries(tutorial_slam2d slam2d_library)
