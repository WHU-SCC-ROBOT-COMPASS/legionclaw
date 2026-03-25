#指定 cmake 的最小版本
cmake_minimum_required(VERSION 3.5)

#设置项目名称
project(lidar_detect)

# project(CUDA_TEST LANGUAGES CUDA CXX)

#设置环境变量编译器类型，编译用到的源文件全部都要放到这里
#set(CMAKE_SYSTEM_NAME Linux)
#set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# set(CMAKE_CROSSCOMPILING TRUE)

set(APP_NAME lidar_detect)
set(THIRD_PATH_HOME_PATH /usr/local/legion/third_party)
set(legion_ALGORITHM_PATH_HOME_PATH /usr/local/legion/legion_algorithm)

option(ROS2_ENABLE "option for ros2 enable" OFF)
if(ROS2_ENABLE)
    set(ros2_interface_DIR ${PROJECT_SOURCE_DIR}/../../../message/ros2/install/ros2_interface/share/ros2_interface/cmake)
	add_definitions(-DROS2_ENABLE)
	find_package(rclcpp REQUIRED)
	find_package(ros2_interface REQUIRED)
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

option(MDCLOG_ENABLE "option for mdc log enable" OFF)
option(GLOG_ENABLE "option for glog enable" ON)
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
	${THIRD_PATH_HOME_PATH}/x64/include/glib
	)

link_directories(
	${THIRD_PATH_HOME_PATH}/x64/lib/boost
	${THIRD_PATH_HOME_PATH}/x64/lib/glib
	${THIRD_PATH_HOME_PATH}/x64/lib/lcm
	${THIRD_PATH_HOME_PATH}/x64/lib/glog
	${THIRD_PATH_HOME_PATH}/x64/lib/gflags
	#${THIRD_PATH_HOME_PATH}/x64/lib/protobuf
)

set(BOOST_LINK_LIBS "-lboost_system  -lboost_signals")

file(GLOB_RECURSE SOURCE_FILES src/*.c src/*.cxx src/*.cpp src/*.cc centerpoint/*.cu centerpoint/*.cpp ../../../common/*.c ../../../common/*.cxx ../../../common/*.cpp ../../../common/*.cc)

# 排除不需要的文件
list(REMOVE_ITEM SOURCE_FILES ${PROJECT_SOURCE_DIR}/../../../common/file/file.cc)
list(REMOVE_ITEM SOURCE_FILES ${PROJECT_SOURCE_DIR}/../../../common/file/file.h)

if(DDS_ENABLE)
find_package(fastcdr HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastcdr)
find_package(fastrtps REQUIRED HINTS ${THIRD_PATH_HOME_PATH}/x64/lib/cmake/fastrtps)
file(GLOB_RECURSE DDS_SOURCE_FILES  ../../../message/dds/*.cxx)
endif()

# pcl and boost related !
find_package(Boost COMPONENTS program_options REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#centerpoint
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

if(${arch} STREQUAL  "aarch64")
  set(TENSORRT_ROOT /usr/src/tensorrt)
  set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

endif()

set(TENSORRT_ROOT /media/zombie/DATA1/Data/TensorRT-8.4.1.5.Linux.x86_64-gnu.cuda-11.6.cudnn8.4/TensorRT-8.4.1.5)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -Wextra -Wall -Wno-deprecated-declarations -O3")
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_87,code=compute_87)
set(TENSORRT_INCLUDE ${TENSORRT_ROOT}/include)
set(TENSORRT_LIB ${TENSORRT_ROOT}/lib)

find_package(CUDA REQUIRED)
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

message("cuda version:"${CUDA_INCLUDE_DIRS})

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  3rdparty/libspconv/include
  include
)

link_directories(
  ${TENSORRT_LIB}
  3rdparty/libspconv/lib/${arch}
)

file(GLOB Centerpoint ${PROJECT_SOURCE_DIR}/centerpoint/*cu)
cuda_add_library(Centerpoint SHARED ${Centerpoint})

add_executable(${APP_NAME}  ${SOURCE_FILES} ${DDS_SOURCE_FILES})

target_link_LIBraries(${APP_NAME}
    libnvinfer.so
    libspconv.so
)

if(ROS_ENABLE)
	target_link_libraries(${APP_NAME}  ${roscpp_LIBRARIES})
endif()
if(ROS2_ENABLE)
	find_package(sensor_msgs)
	ament_target_dependencies(${APP_NAME}  rclcpp ros2_interface sensor_msgs)
endif()
if(LCM_ENABLE)
	target_link_libraries(${APP_NAME}  lcm glib-2.0)
endif()
if(DDS_ENABLE)
target_link_libraries(${APP_NAME}  fastrtps fastcdr)
endif()
target_link_libraries(${APP_NAME}  pthread gflags glog dl ${BOOST_LINK_LIBS} Centerpoint)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
