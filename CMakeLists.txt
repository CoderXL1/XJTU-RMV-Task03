cmake_minimum_required(VERSION 3.10)
project(XJTU-RMV-Task03)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)

add_executable(main src/main.cpp src/frame_processing.cpp src/solver.cpp)

target_link_libraries(main
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog::glog
    Eigen3::Eigen 
)
