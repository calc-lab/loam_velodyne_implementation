cmake_minimum_required(VERSION 2.8)
project(LOAM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        .
        ./DsvLoading)
link_directories(${PCL_LIBRARY_DIRS})

AUX_SOURCE_DIRECTORY(. DIR_SRCS)
AUX_SOURCE_DIRECTORY(./DsvLoading DIR_DL_SRCS)

message("OpenCV_INCLUDE_DIRS = " ${OpenCV_INCLUDE_DIRS})
message("OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBS})

add_executable(LOAM
        ${DIR_SRCS}
        ${DIR_DL_SRCS})
target_link_libraries(LOAM
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})