cmake_minimum_required(VERSION 3.5)

project(lidar_ground_segmentation)

set(APP_NAME lidar_ground_segmentation)
set(THIRD_PATH_HOME_PATH /usr/local/legion/third_party)
set(LEGION_ALGORITHM_PATH_HOME_PATH /usr/local/legion/legion_algorithm)

option(ROS2_ENABLE "option for ros2 enable" OFF)
if(ROS2_ENABLE)
    set(ros2_interface_DIR ${PROJECT_SOURCE_DIR}/../message/ros2/install/ros2_interface/share/ros2_interface/cmake)
	add_definitions(-DROS2_ENABLE)
	find_package(rclcpp REQUIRED)
	find_package(ros2_interface REQUIRED)
	find_package(sensor_msgs REQUIRED)
	include_directories(
	${rclcpp_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/../../../message/ros2/install/ros2_interface/include
	)
endif()

option(ROS_ENABLE "option for ros enable" OFF)
if(ROS_ENABLE)
	add_definitions(-DROS_ENABLE)
	find_package(roscpp REQUIRED)
	include_directories(
	${roscpp_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/../../../message/ros/devel/include
	)
endif()

option(LCM_ENABLE "option for lcm enable" OFF)
if(LCM_ENABLE)
	add_definitions(-DLCM_ENABLE)
	include_directories(
	${PROJECT_SOURCE_DIR}/../../../message/lcm/src/legion
	)
endif()

option(DDS_ENABLE "option for dds enable" OFF)
if(DDS_ENABLE)
	add_definitions(-DDDS_ENABLE)
		include_directories(
	${PROJECT_SOURCE_DIR}/../../../message/dds/src/legion
	)
endif()

option(MDCLOG_ENABLE "option for dds enable" OFF)
option(GLOG_ENABLE "option for dds enable" OFF)
if(MDCLOG_ENABLE)
	add_definitions(-DMDCLOG_ENABLE)
elseif(GLOG_ENABLE)
	add_definitions(-DGLOG_ENABLE)
endif()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++17 -fpermissive -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++17 -fpermissive -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines)
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
    ${PROJECT_SOURCE_DIR}/../../../../..
    ${PROJECT_SOURCE_DIR}/../../../../../..
)

include_directories(
	${THIRD_PATH_HOME_PATH}/x64/include
	${THIRD_PATH_HOME_PATH}/x64/include/eigen3
	)

link_directories(
	${THIRD_PATH_HOME_PATH}/x64/lib/boost
	${THIRD_PATH_HOME_PATH}/x64/lib/lcm
	${THIRD_PATH_HOME_PATH}/x64/lib/glog
	${THIRD_PATH_HOME_PATH}/x64/lib/gflags
	${THIRD_PATH_HOME_PATH}/x64/lib/zlib
	${THIRD_PATH_HOME_PATH}/x64/lib/zip
	${THIRD_PATH_HOME_PATH}/x64/lib/protobuf
)

set(BOOST_LINK_LIBS "-lboost_system  -lboost_signals")

file(GLOB_RECURSE SOURCE_FILES src/*.c src/*.cxx src/*.cpp src/*.cc  ../../../common/*.c ../../../common/*.cxx ../../../common/*.cpp ../../../common/*.cc)

if(DDS_ENABLE)
find_package(fastcdr HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastcdr)
find_package(fastrtps REQUIRED HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastrtps)
file(GLOB_RECURSE DDS_SOURCE_FILES  ../../../message/dds/*.cxx)
endif()
add_executable(${APP_NAME}  ${SOURCE_FILES} ${DDS_SOURCE_FILES})
if(ROS_ENABLE)
	target_link_libraries(${APP_NAME}  ${roscpp_LIBRARIES})
endif()
if(ROS2_ENABLE)
	ament_target_dependencies(${APP_NAME}  rclcpp ros2_interface sensor_msgs)
endif()
if(LCM_ENABLE)
	target_link_libraries(${APP_NAME}  lcm glib-2.0)
endif()
if(DDS_ENABLE)
target_link_libraries(${APP_NAME}  fastrtps fastcdr)
endif()
target_link_libraries(${APP_NAME}  pthread gflags glog z protobuf dl ${BOOST_LINK_LIBS} )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
