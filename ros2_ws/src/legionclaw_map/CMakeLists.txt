cmake_minimum_required(VERSION 3.8)
project(legionclaw_map)
cmake_policy(SET CMP0074 NEW)

# For conditional compilation in FAST-LIO-SAM source
add_definitions(-DROS2_ENABLE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ros2_interface REQUIRED)
find_package(ros2_slam_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# GeographicLib
set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib" ${CMAKE_MODULE_PATH})
find_package(GeographicLib REQUIRED)

set(MAP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/map/src)

include_directories(
  ${MAP_SRC_DIR}
  ${MAP_SRC_DIR}/slam
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
)

file(GLOB_RECURSE MAP_SOURCES
  ${MAP_SRC_DIR}/slam/*.cpp
  ${MAP_SRC_DIR}/slam/*.cc
  ${MAP_SRC_DIR}/apps/*.cpp
)

add_executable(${PROJECT_NAME} ${MAP_SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_components
  sensor_msgs geometry_msgs nav_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs tf2_eigen
  pcl_conversions ros2_interface ros2_slam_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES}
)

if(OpenMP_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
