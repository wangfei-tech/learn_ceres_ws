cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)

find_package(Ceres REQUIRED)
find_package(absl REQUIRED)  # 若你是系统安装的 abseil

##################################################
add_executable(main main.cpp)
target_link_libraries(main
    ${CERES_LIBRARIES}
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
target_include_directories(main PRIVATE ${CERES_INCLUDE_DIRS})
