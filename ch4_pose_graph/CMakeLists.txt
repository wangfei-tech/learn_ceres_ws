cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)

find_package(Ceres REQUIRED)
find_package(absl REQUIRED)  # 若你是系统安装的 abseil

include_directories(
  include
  ${Ceres_INCLUDE_DIRS}
  ${absl_INCLUDE_DIRS}
)
add_executable(pose_graph_2d
    2d_pose_graph.cpp)
target_link_libraries(pose_graph_2d PRIVATE absl::flags absl::flags_parse absl::log absl::check absl::log_initialize Ceres::ceres)
