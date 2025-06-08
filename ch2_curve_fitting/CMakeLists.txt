cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)

find_package(Ceres REQUIRED)
find_package(absl REQUIRED)  # 若你是系统安装的 abseil

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