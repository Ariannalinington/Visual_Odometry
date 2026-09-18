cmake_minimum_required(VERSION 3.16)

project(visual_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(
    visual_odometry
    src/main.cpp
    src/dataset.cpp
    src/data_association.cpp
    src/geometry.cpp
    src/initialization.cpp
    src/projective_icp.cpp
    src/evaluation.cpp
    src/visualization.cpp
)

target_link_libraries(
    visual_odometry
    Eigen3::Eigen
)
