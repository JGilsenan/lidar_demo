cmake_minimum_required(VERSION 2.8.3)
project(lidar_demo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lidar_demo src/lidar_demo.cpp)
target_link_libraries(lidar_demo ${catkin_LIBRARIES})

add_executable(demo src/lidar_demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${catkin_LIBRARIES} lidar_demo)
endif()



