cmake_minimum_required(VERSION 3.5)
project(LOAM)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost 1.6 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        .
        ./DsvLoading
        ./ScanRegistration
        ./LaserMapping)
link_directories(${PCL_LIBRARY_DIRS})

AUX_SOURCE_DIRECTORY(. DIR_SRCS)
AUX_SOURCE_DIRECTORY(./DsvLoading DIR_DL_SRCS)
AUX_SOURCE_DIRECTORY(./ScanRegistration DIR_SR_SRCS)
AUX_SOURCE_DIRECTORY(./LaserMapping DIR_LM_SRCS)

message("OpenCV_INCLUDE_DIRS = " ${OpenCV_INCLUDE_DIRS})
message("OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBS})
message("PCL_INCLUDE_DIRS = " ${PCL_INCLUDE_DIRS})
message("PCL_LIBRARY_DIRS = " ${PCL_LIBRARY_DIRS})
message("Boost_INCLUDE_DIRS = " "${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARY_DIRS = " "${Boost_LIBRARIES}")

add_executable(LOAM
        ${DIR_SRCS}
        ${DIR_DL_SRCS}
        ${DIR_SR_SRCS}
        ${DIR_LM_SRCS})
target_link_libraries(LOAM
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES})