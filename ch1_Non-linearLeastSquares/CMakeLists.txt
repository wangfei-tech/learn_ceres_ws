cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)
cmake_policy(SET CMP0072 NEW)

find_package(Ceres REQUIRED)
message(STATUS "CERES_INCLUDE_DIRS = ${CERES_INCLUDE_DIRS}")
message(STATUS "CERES_LIBRARIES = ${CERES_LIBRARIES}")
find_package(absl REQUIRED)  # 若你是系统安装的 abseil
# 查找必要的依赖包
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

##################################################
add_executable(main main.cpp)
target_link_libraries(main PRIVATE
    Ceres::ceres
    absl::log_initialize
)
##################################################
add_executable(numeric_diff numeric_diff.cpp)
target_link_libraries(numeric_diff
    ${CERES_LIBRARIES}
    absl::log_initialize
)
####################################################
add_executable(analytic_diff analytic_diff.cpp)
target_link_libraries(analytic_diff
    ${CERES_LIBRARIES}
    absl::log_initialize
)
####################################################
add_executable(ad_one_dim_multivariate ad_one_dim_multivariate.cpp)
target_link_libraries(ad_one_dim_multivariate
    ${CERES_LIBRARIES}
    absl::log_initialize
)
####################################################
add_executable(ad_two_dim_multivariate ad_two_dim_multivariate.cpp)
target_link_libraries(ad_two_dim_multivariate
    ${CERES_LIBRARIES}
    absl::log_initialize
)
####################################################
add_executable(ad_three_dim_multivariate ad_three_dim_multivariate.cpp)
target_link_libraries(ad_three_dim_multivariate
    ${CERES_LIBRARIES}
    absl::log_initialize
)
####################################################
add_executable(powerll powerll.cpp)
target_link_libraries(powerll
    ${CERES_LIBRARIES}
    absl::log_initialize
    absl::flags
    absl::log
    absl::strings
    absl::flags_parse
    absl::flags
)
####################################################
add_executable(main_g2o
    main_g2o.cpp  
)
target_link_libraries(main_g2o PRIVATE
    g2o::core
    g2o::stuff
    g2o::types_slam2d
    g2o_solver_dense 
    Eigen3::Eigen
)
####################################################
