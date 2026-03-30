#指定 cmake 的最小版本
cmake_minimum_required(VERSION 3.5)

#设置项目名称
project(multi_lidar_splicing)

#设置环境变量编译器类型
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(CMAKE_CROSSCOMPILING TRUE)

set(APP_NAME multi_lidar_splicing)
set(THIRD_PATH_HOME_PATH /usr/local/legion/third_party)

option(ROS2_ENABLE "option for ros2 enable" OFF)
option(ROS_ENABLE "option for ros enable" OFF)
option(LCM_ENABLE "option for lcm enable" OFF)
option(DDS_ENABLE "option for dds enable" OFF)
option(GLOG_ENABLE "option for glog enable" OFF)

if(NOT ROS2_ENABLE)
  message(WARNING "multi_lidar_splicing is ROS2-only, skip build when ROS2_ENABLE=OFF")
  return()
endif()

set(ros2_interface_DIR ${PROJECT_SOURCE_DIR}/../../../message/ros2/install/ros2_interface/share/ros2_interface/cmake)
add_definitions(-DROS2_ENABLE)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros QUIET)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

if(TARGET JsonCpp::JsonCpp)
  set(JSONCPP_TARGET JsonCpp::JsonCpp)
else()
  find_package(jsoncpp REQUIRED)
  if(TARGET JsonCpp::JsonCpp)
    set(JSONCPP_TARGET JsonCpp::JsonCpp)
  elseif(TARGET jsoncpp_lib)
    set(JSONCPP_TARGET jsoncpp_lib)
  else()
    message(FATAL_ERROR "JsonCpp target not found")
  endif()
endif()

set(CPP_STANDARD_FLAG -std=c++17)
if (CMAKE_BUILD_TYPE MATCHES Debug)
  set(DEFAULT_COMPILE ${CPP_STANDARD_FLAG} -fpermissive -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
  set(DEFAULT_COMPILE ${CPP_STANDARD_FLAG} -fpermissive -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
endif()
add_compile_options(${DEFAULT_COMPILE})

macro(LISTSUBDIR result currentdir)
file(GLOB_RECURSE children LIST_DIRECTORIES true  ${currentdir}/*)
set(dirlist ${currentdir})
foreach(child ${children})
  if(IS_DIRECTORY ${child})
    list(APPEND dirlist ${child})
  endif()
endforeach()
set(${result} ${dirlist})
endmacro()

LISTSUBDIR(SOURCE_DIRS src)
include_directories(${SOURCE_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}/..
  ${PROJECT_SOURCE_DIR}/../..
  ${PROJECT_SOURCE_DIR}/../../..
  ${PROJECT_SOURCE_DIR}/../../../..
  ${THIRD_PATH_HOME_PATH}/arm64/include
  ${THIRD_PATH_HOME_PATH}/arm64/include/eigen3
  ${rclcpp_INCLUDE_DIRS}
  /opt/ros/$ENV{ROS_DISTRO}/include
  /opt/ros/$ENV{ROS_DISTRO}/include/sensor_msgs
  ${sensor_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if(pcl_ros_FOUND)
  include_directories(${pcl_ros_INCLUDE_DIRS})
endif()

link_directories(
  ${THIRD_PATH_HOME_PATH}/arm64/lib/glog
  ${THIRD_PATH_HOME_PATH}/arm64/lib/gflags
  ${THIRD_PATH_HOME_PATH}/arm64/lib/zlib
)

file(GLOB_RECURSE SOURCE_FILES src/*.c src/*.cxx src/*.cpp src/*.cc)
add_executable(${APP_NAME} ${SOURCE_FILES})

ament_target_dependencies(${APP_NAME} rclcpp sensor_msgs message_filters)
ament_target_dependencies(${APP_NAME} pcl_conversions)

target_link_libraries(${APP_NAME}
  ${PCL_LIBRARIES}
  ${JSONCPP_TARGET}
  pthread
  gflags
  glog
  z
)

if(pcl_ros_FOUND)
  target_link_libraries(${APP_NAME} ${pcl_ros_LIBRARIES})
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
