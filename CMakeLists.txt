cmake_minimum_required(VERSION 2.8)
set(OpenCV_DIR ../VisualSlam/ThirdParty/opencv-4.3.0/build)
find_package( OpenCV REQUIRED)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-g -O0 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
#Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
AUX_SOURCE_DIRECTORY(./src DIR_SRCS)
include_directories(./inc)
include_directories (${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories ("../VisualSlam/ThirdParty/Sophus-master/")

add_executable(steroslam SteroSlam.cpp ${DIR_SRCS})

target_link_libraries(steroslam ${OpenCV_LIBS} ${G2O_LIBS})
