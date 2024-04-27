cmake_minimum_required(VERSION 3.0.2)
project(PGMF_filter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PGMF_filter
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(PGMF_filter 
    src/PGMF_filter.cpp
)
target_link_libraries(PGMF_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${CERES_LIBRARIES}
)
