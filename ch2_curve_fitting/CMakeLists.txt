cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)
cmake_policy(SET CMP0072 NEW)

find_package(Ceres REQUIRED)
find_package(absl REQUIRED)  # 若你是系统安装的 abseil
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

##################################################
add_executable(curve_fitting curve_fitting.cpp)
target_link_libraries(curve_fitting
    ${CERES_LIBRARIES}
    absl::log_initialize
)
###################################################
add_executable(robust_curve_fitting robust_curve_fitting.cpp)
target_link_libraries(robust_curve_fitting
    ${CERES_LIBRARIES}
    absl::log_initialize
)
###################################################
add_executable(curve_fitting_g2o
curve_fitting_g2o.cpp  
)
target_link_libraries(curve_fitting_g2o PRIVATE
    g2o::core
    g2o::stuff
    g2o::types_slam2d
    g2o_solver_dense 
    Eigen3::Eigen
)
####################################################