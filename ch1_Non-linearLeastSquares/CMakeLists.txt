cmake_minimum_required(VERSION 3.10)
project(my_ceres_example)

find_package(Ceres REQUIRED)
find_package(absl REQUIRED)  # 若你是系统安装的 abseil

add_executable(main main.cpp)
target_link_libraries(main
    ${CERES_LIBRARIES}
    absl::log_initialize
)
target_include_directories(main PRIVATE ${CERES_INCLUDE_DIRS})
