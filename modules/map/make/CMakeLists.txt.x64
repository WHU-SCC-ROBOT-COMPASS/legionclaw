cmake_minimum_required(VERSION 3.5)

project(map)

set(APP_NAME map)
set(THIRD_PATH_HOME_PATH /usr/local/legionclaw/third_party)
set(LEGION_ALGORITHM_PATH_HOME_PATH /usr/local/legionclaw/legionclaw_algorithm)

# =====================================================================
# Middleware options
# =====================================================================
option(ROS2_ENABLE "option for ros2 enable" OFF)
if(ROS2_ENABLE)
    set(ros2_interface_DIR ${PROJECT_SOURCE_DIR}/../message/ros2/install/ros2_interface/share/ros2_interface/cmake)
    set(ros2_slam_msgs_DIR ${PROJECT_SOURCE_DIR}/../message/ros2/install/ros2_slam_msgs/share/ros2_slam_msgs/cmake)
    add_definitions(-DROS2_ENABLE)
    find_package(rclcpp REQUIRED)
    find_package(ros2_interface REQUIRED)
    find_package(ros2_slam_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_eigen REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(pcl_conversions REQUIRED)
    find_package(visualization_msgs REQUIRED)
    include_directories(
        ${rclcpp_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../message/ros2/install/ros2_interface/include
        ${PROJECT_SOURCE_DIR}/../message/ros2/install/ros2_slam_msgs/include
    )
endif()

option(ROS_ENABLE "option for ros enable" OFF)
if(ROS_ENABLE)
    add_definitions(-DROS_ENABLE)
    find_package(roscpp REQUIRED)
    find_package(pcl_ros REQUIRED)
    include_directories(
        ${roscpp_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../message/ros/devel/include
    )
endif()

option(DDS_ENABLE "option for dds enable" OFF)
if(DDS_ENABLE)
    add_definitions(-DDDS_ENABLE)
    find_package(fastcdr HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastcdr)
    find_package(fastrtps REQUIRED HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastrtps)
    include_directories(
        ${PROJECT_SOURCE_DIR}/../message/dds/src/legion
    )
endif()

option(LCM_ENABLE "option for lcm enable" OFF)
if(LCM_ENABLE)
    add_definitions(-DLCM_ENABLE)
    include_directories(
        ${PROJECT_SOURCE_DIR}/../message/lcm/src/legion
    )
endif()

option(GLOG_ENABLE "option for glog enable" OFF)
option(MDCLOG_ENABLE "option for mdc log enable" OFF)
if(MDCLOG_ENABLE)
    add_definitions(-DMDCLOG_ENABLE)
elseif(GLOG_ENABLE)
    add_definitions(-DGLOG_ENABLE)
endif()

option(OPENMP_ENABLE "option for OpenMP parallel" ON)
if(OPENMP_ENABLE)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

# =====================================================================
# Compiler settings
# =====================================================================
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++17 -fpermissive -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++17 -fpermissive -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
endif()
add_compile_options(${DEFAULT_COMPILE})

# =====================================================================
# Find packages
# =====================================================================
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
)

# =====================================================================
# Include directories
# =====================================================================
include_directories(
    ${PROJECT_SOURCE_DIR}/..
    ${PROJECT_SOURCE_DIR}/../..
)

include_directories(
    ${THIRD_PATH_HOME_PATH}/x64/include
    ${THIRD_PATH_HOME_PATH}/x64/include/eigen3
    ${THIRD_PATH_HOME_PATH}/x64/include/Geographic
)

# SLAM source (FAST-LIO-SAM core algorithms)
include_directories(
    ${PROJECT_SOURCE_DIR}/src/slam
    ${PROJECT_SOURCE_DIR}/src/slam/IKFoM_toolkit
    ${PROJECT_SOURCE_DIR}/src/slam/IKFoM_toolkit/esekfom
    ${PROJECT_SOURCE_DIR}/src/slam/IKFoM_toolkit/mtk
    ${PROJECT_SOURCE_DIR}/src/slam/IKFoM_toolkit/mtk/src
    ${PROJECT_SOURCE_DIR}/src/slam/IKFoM_toolkit/mtk/src/types
    ${PROJECT_SOURCE_DIR}/src/slam/ikd-Tree
)

# =====================================================================
# Source files
# =====================================================================
file(GLOB_RECURSE MODULE_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cc
    ${PROJECT_SOURCE_DIR}/src/*.cxx
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/slam/*.cpp
    ${PROJECT_SOURCE_DIR}/src/slam/*.cc
    ${PROJECT_SOURCE_DIR}/src/slam/*.cxx
)

file(GLOB_RECURSE COMMON_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/../common/src/*.cpp
    ${PROJECT_SOURCE_DIR}/../common/src/*.cc
)

set(ALL_SOURCE_FILES ${MODULE_SOURCE_FILES})

if(DDS_ENABLE)
    file(GLOB_RECURSE DDS_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/../message/dds/*.cxx
    )
    set(ALL_SOURCE_FILES ${ALL_SOURCE_FILES} ${DDS_SOURCE_FILES})
endif()

# =====================================================================
# Executable
# =====================================================================
add_executable(${APP_NAME} ${ALL_SOURCE_FILES})

set(BOOST_LINK_LIBS -lboost_system)
link_directories(
    ${THIRD_PATH_HOME_PATH}/x64/lib/boost
    ${THIRD_PATH_HOME_PATH}/x64/lib/glog
    ${THIRD_PATH_HOME_PATH}/x64/lib/gflags
    ${THIRD_PATH_HOME_PATH}/x64/lib/zlib
)

target_link_libraries(${APP_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    pthread
    gflags
    glog
    ${BOOST_LINK_LIBS}
    dl
)

    target_link_libraries(${APP_NAME} ${GeographicLib_LIBRARIES})

if(OPENMP_FOUND)
    target_link_libraries(${APP_NAME} OpenMP::OpenMP_CXX)
endif()

if(ROS_ENABLE)
    target_link_libraries(${APP_NAME} ${roscpp_LIBRARIES} ${pcl_ros_LIBRARIES})
endif()

if(ROS2_ENABLE)
    ament_target_dependencies(${APP_NAME}
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_conversions
        ros2_interface
        ros2_slam_msgs
    )
endif()

if(LCM_ENABLE)
    target_link_libraries(${APP_NAME} lcm glib-2.0)
endif()

if(DDS_ENABLE)
    target_link_libraries(${APP_NAME} fastrtps fastcdr)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

install(TARGETS ${APP_NAME}
  RUNTIME DESTINATION bin
)
