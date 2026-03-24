cmake_minimum_required(VERSION 3.10)
project(multi_lidar_splicing)

set(APP_NAME multi_lidar_splicing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ROS2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# 直接查找pcl_ros（包含pcl_conversions）
find_package(pcl_ros QUIET)
if(NOT pcl_ros_FOUND)
    message(WARNING "pcl_ros not found. Please install: sudo apt install ros-foxy-pcl-ros")
endif()

# 3rd-party
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

# JsonCpp is sometimes already provided by the ROS overlay; avoid redefining it
set(JSONCPP_TARGET "")
if(TARGET JsonCpp::JsonCpp)
    set(JSONCPP_TARGET JsonCpp::JsonCpp)
else()
    find_package(jsoncpp REQUIRED)
    if(TARGET JsonCpp::JsonCpp)
        set(JSONCPP_TARGET JsonCpp::JsonCpp)
    elseif(TARGET jsoncpp_lib)
        set(JSONCPP_TARGET jsoncpp_lib)
    endif()
endif()
if(NOT JSONCPP_TARGET)
    message(FATAL_ERROR "JsonCpp target not found")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
)

# 添加pcl_ros的包含目录
if(pcl_ros_FOUND)
    include_directories(${pcl_ros_INCLUDE_DIRS})
endif()

# 基于rclcpp的路径添加ROS2的默认include路径
get_filename_component(ROS_INSTALL_PREFIX "${rclcpp_INCLUDE_DIRS}" DIRECTORY)
include_directories(${ROS_INSTALL_PREFIX}/include)

# 统一把可执行文件放到 build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

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

file(GLOB_RECURSE SOURCE_FILES src/*.c src/*.cxx src/*.cpp src/*.cc)

add_executable(${APP_NAME} ${SOURCE_FILES})

target_link_libraries(${APP_NAME}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${PCL_LIBRARIES}
  ${JSONCPP_TARGET}
)

# 链接pcl_ros库（如果找到）
if(pcl_ros_FOUND)
    target_link_libraries(${APP_NAME} ${pcl_ros_LIBRARIES})
endif()

# 确保包含ROS2的路径（基于rclcpp的路径）
target_include_directories(${APP_NAME} PRIVATE ${rclcpp_INCLUDE_DIRS})
