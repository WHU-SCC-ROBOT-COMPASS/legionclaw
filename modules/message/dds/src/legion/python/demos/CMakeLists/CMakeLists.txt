cmake_minimum_required(VERSION 3.5) 

if(NOT CMAKE_VERSION VERSION_LESS 3.0) 
    cmake_policy(SET CMP0048 NEW) 
endif() 

# SWIG: use standard target name. 
if(POLICY CMP0078) 
    cmake_policy(SET CMP0078 NEW) 
endif() 

# SWIG: use SWIG_MODULE_NAME property. 
if(POLICY CMP0086) 
    cmake_policy(SET CMP0086 NEW) 
endif() 

project(dds_interface) 

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_BUILD_TYPE Release)

############################################################################### 
# Dependencies 
############################################################################### 

FIND_PACKAGE(SWIG REQUIRED) 
INCLUDE(${SWIG_USE_FILE}) 
SET(CMAKE_SWIG_FLAGS "") 

FIND_PACKAGE(PythonLibs) 
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH}) 

find_package(fastcdr REQUIRED HINTS /usr/local/legionclaw/third_party/x64/lib/cmake/fastcdr) 
find_package(foonathan_memory  HINTS /usr/local/legionclaw/third_party/x64/lib/foonathan_memory/cmake) 
find_package(fastrtps REQUIRED HINTS /usr/local/legionclaw/third_party/x64/lib/cmake/fastrtps) 


set(DDS_PATH ../../../dds_interface) 
############################################################################### 
# Compile python wrapper with SWIG 
############################################################################### 

include_directories(../../../dds_interface/) 
include_directories(../../fastdds_wrapper/) 
message(STATUS "Configuring FastDDS wrapper for Python...") 
file(GLOB DDS_CXX ${DDS_PATH}/*.cxx) 
file(GLOB DDS_CPP ${DDS_PATH}/*..cpp) 
set(SWIG_SOURCE ../../swig) 
SET_SOURCE_FILES_PROPERTIES( 
    ${SWIG_SOURCE}/fastdds_lane_info_extend.i 
    ${SWIG_SOURCE}/fastdds_local_map.i 
    ${SWIG_SOURCE}/fastdds_header.i 
    ${SWIG_SOURCE}/fastdds_time.i 
    ${SWIG_SOURCE}/fastdds_status.i 
    ${SWIG_SOURCE}/fastdds_point_enu.i 
    ${SWIG_SOURCE}/fastdds_point_basic.i 
    ${SWIG_SOURCE}/fastdds_point_llh.i 
    ${SWIG_SOURCE}/fastdds_point_2d.i 
    ${SWIG_SOURCE}/fastdds_point_3d.i 
    ${SWIG_SOURCE}/fastdds_quaternion.i 
    ${SWIG_SOURCE}/fastdds_polygon_3d.i 
    ${SWIG_SOURCE}/fastdds_polygon_2d.i 
    ${SWIG_SOURCE}/fastdds_image_key_point.i 
    ${SWIG_SOURCE}/fastdds_sl_point.i 
    ${SWIG_SOURCE}/fastdds_sl_boundary.i 
    ${SWIG_SOURCE}/fastdds_frenet_frame_point.i 
    ${SWIG_SOURCE}/fastdds_speed_point.i 
    ${SWIG_SOURCE}/fastdds_path_point.i 
    ${SWIG_SOURCE}/fastdds_path.i 
    ${SWIG_SOURCE}/fastdds_trajectory_point.i 
    ${SWIG_SOURCE}/fastdds_trajectory.i 
    ${SWIG_SOURCE}/fastdds_vehicle_motion_point.i 
    ${SWIG_SOURCE}/fastdds_vehicle_motion.i 
    ${SWIG_SOURCE}/fastdds_gaussian_info.i 
    ${SWIG_SOURCE}/fastdds_vehicle_signal.i 
    ${SWIG_SOURCE}/fastdds_pad_message.i 
    ${SWIG_SOURCE}/fastdds_stories.i 
    ${SWIG_SOURCE}/fastdds_fault.i 
    ${SWIG_SOURCE}/fastdds_faults.i 
    ${SWIG_SOURCE}/fastdds_event.i 
    ${SWIG_SOURCE}/fastdds_events.i 
    ${SWIG_SOURCE}/fastdds_estop.i 
    ${SWIG_SOURCE}/fastdds_matrix_3d.i 
    ${SWIG_SOURCE}/fastdds_pose.i 
    ${SWIG_SOURCE}/fastdds_uncertainty.i 
    ${SWIG_SOURCE}/fastdds_chassis.i 
    ${SWIG_SOURCE}/fastdds_control_command.i 
    ${SWIG_SOURCE}/fastdds_wheel_info.i 
    ${SWIG_SOURCE}/fastdds_control_analysis.i 
    ${SWIG_SOURCE}/fastdds_vehicle_param.i 
    ${SWIG_SOURCE}/fastdds_vehicle_config.i 
    ${SWIG_SOURCE}/fastdds_prediction_obstacles.i 
    ${SWIG_SOURCE}/fastdds_location.i 
    ${SWIG_SOURCE}/fastdds_odometry.i 
    ${SWIG_SOURCE}/fastdds_obstacle_list.i 
    ${SWIG_SOURCE}/fastdds_lane_list.i 
    ${SWIG_SOURCE}/fastdds_traffic_light_msg.i 
    ${SWIG_SOURCE}/fastdds_ultrasonic.i 
    ${SWIG_SOURCE}/fastdds_radar_obstacle_list_msg.i 
    ${SWIG_SOURCE}/fastdds_point_cloud.i 
    ${SWIG_SOURCE}/fastdds_radar_obstacle.i 
    ${SWIG_SOURCE}/fastdds_ultrasonic_obstacle.i 
    ${SWIG_SOURCE}/fastdds_point_xyzirt.i 
    ${SWIG_SOURCE}/fastdds_compressed_image.i 
    ${SWIG_SOURCE}/fastdds_image.i 
    ${SWIG_SOURCE}/fastdds_gnss.i 
    ${SWIG_SOURCE}/fastdds_imu.i 
    ${SWIG_SOURCE}/fastdds_ins.i 
    ${SWIG_SOURCE}/fastdds_obu_cmd_msg.i 
    ${SWIG_SOURCE}/fastdds_routing_request.i 
    ${SWIG_SOURCE}/fastdds_obu_cmd.i 
    ${SWIG_SOURCE}/fastdds_parking_info.i 
    ${SWIG_SOURCE}/fastdds_key_point.i 
    ${SWIG_SOURCE}/fastdds_sensor_calibrator.i 
    ${SWIG_SOURCE}/fastdds_traffic_light.i 
    ${SWIG_SOURCE}/fastdds_stop_point.i 
    ${SWIG_SOURCE}/fastdds_lane_point.i 
    ${SWIG_SOURCE}/fastdds_lane_info.i 
    ${SWIG_SOURCE}/fastdds_obstacle_intent.i 
    ${SWIG_SOURCE}/fastdds_routing_response.i 
    ${SWIG_SOURCE}/fastdds_stop_info.i 
    ${SWIG_SOURCE}/fastdds_adc_trajectory.i 
    ${SWIG_SOURCE}/fastdds_bbox_2d.i 
    ${SWIG_SOURCE}/fastdds_obstacle.i 
    ${SWIG_SOURCE}/fastdds_rss_info.i 
    ${SWIG_SOURCE}/fastdds_obstacle_priority.i 
    ${SWIG_SOURCE}/fastdds_planning_cmd.i 
    ${SWIG_SOURCE}/fastdds_prediction_obstacle.i 
    ${SWIG_SOURCE}/fastdds_trajectory_in_prediction.i 
    ${SWIG_SOURCE}/fastdds_trajectory_point_in_prediction.i 
    ${SWIG_SOURCE}/fastdds_prediction_trajectory_point.i 
    ${SWIG_SOURCE}/fastdds_perception_obstacle.i 
    ${SWIG_SOURCE}/fastdds_obstacle_feature.i 
    ${SWIG_SOURCE}/fastdds_lane_line.i 
    ${SWIG_SOURCE}/fastdds_lane_line_cubic_curve.i 
    ${SWIG_SOURCE}/fastdds_end_points.i 
    ${SWIG_SOURCE}/fastdds_free_space.i 
    ${SWIG_SOURCE}/fastdds_holistic_path_prediction.i 
    ${SWIG_SOURCE}/fastdds_road_mark.i 
    ${SWIG_SOURCE}/fastdds_traffic_light_debug.i 
    ${SWIG_SOURCE}/fastdds_traffic_light_box.i 
    ${SWIG_SOURCE}/fastdds_image_rect.i 
    ${SWIG_SOURCE}/fastdds_radar_state.i 
    ${SWIG_SOURCE}/fastdds_radar_state_error.i 
    ${SWIG_SOURCE}/fastdds_radar_state_mode.i 
    ${SWIG_SOURCE}/fastdds_sotif_monitor_result.i 
    ${SWIG_SOURCE}/fastdds_region.i 
    ${SWIG_SOURCE}/fastdds_grid.i 
    ${SWIG_SOURCE}/fastdds_camera_parking_info.i 
    ${SWIG_SOURCE}/fastdds_camera_parking_info_list.i 
    ${SWIG_SOURCE}/fastdds_security_decision.i 
    ${SWIG_SOURCE}/fastdds_warning_command.i 
    ${SWIG_SOURCE}/fastdds_trajectory_limit_command.i 
    ${SWIG_SOURCE}/fastdds_parking_info_list.i 
    ${SWIG_SOURCE}/fastdds_comm_command.i 
    ${SWIG_SOURCE}/fastdds_command_respond.i 
    ${SWIG_SOURCE}/fastdds_module_status.i 
    ${SWIG_SOURCE}/fastdds_command.i 
    ${SWIG_SOURCE}/fastdds_message.i 
    ${SWIG_SOURCE}/fastdds_planning_analysis.i 
    ${SWIG_SOURCE}/fastdds_time_consume.i 
    ${SWIG_SOURCE}/fastdds_planning_parking_debug.i 
    ${SWIG_SOURCE}/fastdds_trajectory_array.i 
    ${SWIG_SOURCE}/fastdds_vehicle_state.i 
    ${SWIG_SOURCE}/fastdds_drivable_region.i 
    ${SWIG_SOURCE}/fastdds_obstacle_interactive_tag.i 
    ${SWIG_SOURCE}/fastdds_twist.i 
    ${SWIG_SOURCE}/fastdds_parking_state_display.i 
    ${SWIG_SOURCE}/fastdds_parking_roi.i 
    ${SWIG_SOURCE}/fastdds_point_2d_list.i 
    ${SWIG_SOURCE}/fastdds_parking_out_info.i 
    ${SWIG_SOURCE}/fastdds_junction_info.i 
    ${SWIG_SOURCE}/fastdds_traffic_events.i 
    ${SWIG_SOURCE}/fastdds_limit_speed_info.i 
    ${SWIG_SOURCE}/fastdds_log.i 
    ${SWIG_SOURCE}/fastdds_pavementype.i 
    ${SWIG_SOURCE}/fastdds_wl_constraint_info_list.i 
    ${SWIG_SOURCE}/fastdds_wl_constraint_info.i 
    ${SWIG_SOURCE}/fastdds_uss_obstacle_list.i 
    ${SWIG_SOURCE}/fastdds_uss_obstacle.i 
    ${SWIG_SOURCE}/fastdds_uss_parking_info_list.i 
    ${SWIG_SOURCE}/fastdds_uss_parking_info.i 
    ${SWIG_SOURCE}/fastdds_camera_parking_stopper.i 
    ${SWIG_SOURCE}/fastdds_parking_stopper.i 
    ${SWIG_SOURCE}/fastdds_diagnostic_array.i 
    ${SWIG_SOURCE}/fastdds_diagnostic_status.i 
    ${SWIG_SOURCE}/fastdds_key_values.i 
    ${SWIG_SOURCE}/fastdds_alarm_message.i 
    ${SWIG_SOURCE}/fastdds_lanelet_info.i 
    ${SWIG_SOURCE}/fastdds_global_route_msg.i 
    ${SWIG_SOURCE}/fastdds_route_fusion_info.i 
    ${SWIG_SOURCE}/fastdds_road_mark_list.i 
    ${SWIG_SOURCE}/fastdds_hmi_obu_cmd.i 
    ${SWIG_SOURCE}/fastdds_hmi_obu_cmd_msg.i 
    ${SWIG_SOURCE}/fastdds_hmi_parking_state_display.i 
    ${SWIG_SOURCE}/fastdds_hmi_parking_info.i 
    ${SWIG_SOURCE}/fastdds_hmi_parking_info_list.i 
    ${SWIG_SOURCE}/fastdds_hmi_vehicle_msg.i 
    ${SWIG_SOURCE}/fastdds_hmi_obstacle.i 
    ${SWIG_SOURCE}/fastdds_hmi_obstacle_list.i 
    ${SWIG_SOURCE}/fastdds_hmi_trajectory_point.i 
    ${SWIG_SOURCE}/fastdds_hmi_trajectory.i 
    ${SWIG_SOURCE}/fastdds_hmi_diagnostic_status.i 
    ${SWIG_SOURCE}/fastdds_hmi_diagnostic_array.i 
    ${SWIG_SOURCE}/fastdds_guide_info.i 
    ${SWIG_SOURCE}/fastdds_guide_road.i 
    ${SWIG_SOURCE}/fastdds_curvature_info.i 
    ${DDS_CXX} 
    ${DDS_CPP} 
    PROPERTIES CPLUSPLUS ON) 

INCLUDE_DIRECTORIES( 
    fastdds_lane_info_extend 
    fastdds_local_map 
    fastdds_header 
    fastdds_time 
    fastdds_status 
    fastdds_point_enu 
    fastdds_point_basic 
    fastdds_point_llh 
    fastdds_point_2d 
    fastdds_point_3d 
    fastdds_quaternion 
    fastdds_polygon_3d 
    fastdds_polygon_2d 
    fastdds_image_key_point 
    fastdds_sl_point 
    fastdds_sl_boundary 
    fastdds_frenet_frame_point 
    fastdds_speed_point 
    fastdds_path_point 
    fastdds_path 
    fastdds_trajectory_point 
    fastdds_trajectory 
    fastdds_vehicle_motion_point 
    fastdds_vehicle_motion 
    fastdds_gaussian_info 
    fastdds_vehicle_signal 
    fastdds_pad_message 
    fastdds_stories 
    fastdds_fault 
    fastdds_faults 
    fastdds_event 
    fastdds_events 
    fastdds_estop 
    fastdds_matrix_3d 
    fastdds_pose 
    fastdds_uncertainty 
    fastdds_chassis 
    fastdds_control_command 
    fastdds_wheel_info 
    fastdds_control_analysis 
    fastdds_vehicle_param 
    fastdds_vehicle_config 
    fastdds_prediction_obstacles 
    fastdds_location 
    fastdds_odometry 
    fastdds_obstacle_list 
    fastdds_lane_list 
    fastdds_traffic_light_msg 
    fastdds_ultrasonic 
    fastdds_radar_obstacle_list_msg 
    fastdds_point_cloud 
    fastdds_radar_obstacle 
    fastdds_ultrasonic_obstacle 
    fastdds_point_xyzirt 
    fastdds_compressed_image 
    fastdds_image 
    fastdds_gnss 
    fastdds_imu 
    fastdds_ins 
    fastdds_obu_cmd_msg 
    fastdds_routing_request 
    fastdds_obu_cmd 
    fastdds_parking_info 
    fastdds_key_point 
    fastdds_sensor_calibrator 
    fastdds_traffic_light 
    fastdds_stop_point 
    fastdds_lane_point 
    fastdds_lane_info 
    fastdds_obstacle_intent 
    fastdds_routing_response 
    fastdds_stop_info 
    fastdds_adc_trajectory 
    fastdds_bbox_2d 
    fastdds_obstacle 
    fastdds_rss_info 
    fastdds_obstacle_priority 
    fastdds_planning_cmd 
    fastdds_prediction_obstacle 
    fastdds_trajectory_in_prediction 
    fastdds_trajectory_point_in_prediction 
    fastdds_prediction_trajectory_point 
    fastdds_perception_obstacle 
    fastdds_obstacle_feature 
    fastdds_lane_line 
    fastdds_lane_line_cubic_curve 
    fastdds_end_points 
    fastdds_free_space 
    fastdds_holistic_path_prediction 
    fastdds_road_mark 
    fastdds_traffic_light_debug 
    fastdds_traffic_light_box 
    fastdds_image_rect 
    fastdds_radar_state 
    fastdds_radar_state_error 
    fastdds_radar_state_mode 
    fastdds_sotif_monitor_result 
    fastdds_region 
    fastdds_grid 
    fastdds_camera_parking_info 
    fastdds_camera_parking_info_list 
    fastdds_security_decision 
    fastdds_warning_command 
    fastdds_trajectory_limit_command 
    fastdds_parking_info_list 
    fastdds_comm_command 
    fastdds_command_respond 
    fastdds_module_status 
    fastdds_command 
    fastdds_message 
    fastdds_planning_analysis 
    fastdds_time_consume 
    fastdds_planning_parking_debug 
    fastdds_trajectory_array 
    fastdds_vehicle_state 
    fastdds_drivable_region 
    fastdds_obstacle_interactive_tag 
    fastdds_twist 
    fastdds_parking_state_display 
    fastdds_parking_roi 
    fastdds_point_2d_list 
    fastdds_parking_out_info 
    fastdds_junction_info 
    fastdds_traffic_events 
    fastdds_limit_speed_info 
    fastdds_log 
    fastdds_pavementype 
    fastdds_wl_constraint_info_list 
    fastdds_wl_constraint_info 
    fastdds_uss_obstacle_list 
    fastdds_uss_obstacle 
    fastdds_uss_parking_info_list 
    fastdds_uss_parking_info 
    fastdds_camera_parking_stopper 
    fastdds_parking_stopper 
    fastdds_diagnostic_array 
    fastdds_diagnostic_status 
    fastdds_key_values 
    fastdds_alarm_message 
    fastdds_lanelet_info 
    fastdds_global_route_msg 
    fastdds_route_fusion_info 
    fastdds_road_mark_list 
    fastdds_hmi_obu_cmd 
    fastdds_hmi_obu_cmd_msg 
    fastdds_hmi_parking_state_display 
    fastdds_hmi_parking_info 
    fastdds_hmi_parking_info_list 
    fastdds_hmi_vehicle_msg 
    fastdds_hmi_obstacle 
    fastdds_hmi_obstacle_list 
    fastdds_hmi_trajectory_point 
    fastdds_hmi_trajectory 
    fastdds_hmi_diagnostic_status 
    fastdds_hmi_diagnostic_array 
    fastdds_guide_info 
    fastdds_guide_road 
    fastdds_curvature_info 
) 

SWIG_ADD_LIBRARY(fastdds_lane_info_extend 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_info_extend.i ${DDS_PATH}/LanePoint.cxx ${DDS_PATH}/LanePointPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/LaneInfoExtend.cxx ${DDS_PATH}/LaneInfoExtendPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_info_extend ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_local_map 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_local_map.i ${DDS_PATH}/LanePoint.cxx ${DDS_PATH}/LanePointPubSubTypes.cxx ${DDS_PATH}/LocalMap.cxx ${DDS_PATH}/LocalMapPubSubTypes.cxx ${DDS_PATH}/LaneInfoExtend.cxx ${DDS_PATH}/LaneInfoExtendPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_local_map ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_header 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_header.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_header ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_time 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_time.i ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_time ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_status 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_status.i ${DDS_PATH}/Status.cxx ${DDS_PATH}/StatusPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_status ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_enu 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_enu.i ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_enu ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_basic 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_basic.i ${DDS_PATH}/PointBasic.cxx ${DDS_PATH}/PointBasicPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_basic ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_llh 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_llh.i ${DDS_PATH}/PointLLH.cxx ${DDS_PATH}/PointLLHPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_llh ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_2d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_2d.i ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_2d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_3d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_3d.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_3d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_quaternion 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_quaternion.i ${DDS_PATH}/Quaternion.cxx ${DDS_PATH}/QuaternionPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_quaternion ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_polygon_3d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_polygon_3d.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_polygon_3d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_polygon_2d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_polygon_2d.i ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Polygon2D.cxx ${DDS_PATH}/Polygon2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_polygon_2d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_image_key_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_image_key_point.i ${DDS_PATH}/ImageKeyPoint.cxx ${DDS_PATH}/ImageKeyPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_image_key_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_sl_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_sl_point.i ${DDS_PATH}/SLPoint.cxx ${DDS_PATH}/SLPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_sl_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_sl_boundary 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_sl_boundary.i ${DDS_PATH}/SLBoundary.cxx ${DDS_PATH}/SLBoundaryPubSubTypes.cxx ${DDS_PATH}/SLPoint.cxx ${DDS_PATH}/SLPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_sl_boundary ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_frenet_frame_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_frenet_frame_point.i ${DDS_PATH}/FrenetFramePoint.cxx ${DDS_PATH}/FrenetFramePointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_frenet_frame_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_speed_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_speed_point.i ${DDS_PATH}/SpeedPoint.cxx ${DDS_PATH}/SpeedPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_speed_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_path_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_path_point.i ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_path_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_path 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_path.i ${DDS_PATH}/Path.cxx ${DDS_PATH}/PathPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_path ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory_point.i ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory.i ${DDS_PATH}/Trajectory.cxx ${DDS_PATH}/TrajectoryPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_motion_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_motion_point.i ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/VehicleMotionPoint.cxx ${DDS_PATH}/VehicleMotionPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_motion_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_motion 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_motion.i ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/VehicleMotion.cxx ${DDS_PATH}/VehicleMotionPubSubTypes.cxx ${DDS_PATH}/VehicleMotionPoint.cxx ${DDS_PATH}/VehicleMotionPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_motion ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_gaussian_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_gaussian_info.i ${DDS_PATH}/GaussianInfo.cxx ${DDS_PATH}/GaussianInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_gaussian_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_signal 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_signal.i ${DDS_PATH}/VehicleSignal.cxx ${DDS_PATH}/VehicleSignalPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_signal ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_pad_message 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_pad_message.i ${DDS_PATH}/PadMessage.cxx ${DDS_PATH}/PadMessagePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_pad_message ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_stories 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_stories.i ${DDS_PATH}/Stories.cxx ${DDS_PATH}/StoriesPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_stories ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_fault 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_fault.i ${DDS_PATH}/Fault.cxx ${DDS_PATH}/FaultPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_fault ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_faults 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_faults.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Fault.cxx ${DDS_PATH}/FaultPubSubTypes.cxx ${DDS_PATH}/Faults.cxx ${DDS_PATH}/FaultsPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_faults ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_event 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_event.i ${DDS_PATH}/Event.cxx ${DDS_PATH}/EventPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_event ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_events 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_events.i ${DDS_PATH}/Events.cxx ${DDS_PATH}/EventsPubSubTypes.cxx ${DDS_PATH}/Event.cxx ${DDS_PATH}/EventPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_events ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_estop 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_estop.i ${DDS_PATH}/EStop.cxx ${DDS_PATH}/EStopPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_estop ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_matrix_3d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_matrix_3d.i ${DDS_PATH}/Matrix3D.cxx ${DDS_PATH}/Matrix3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_matrix_3d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_pose 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_pose.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Pose.cxx ${DDS_PATH}/PosePubSubTypes.cxx ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ${DDS_PATH}/Quaternion.cxx ${DDS_PATH}/QuaternionPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_pose ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_uncertainty 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_uncertainty.i ${DDS_PATH}/Uncertainty.cxx ${DDS_PATH}/UncertaintyPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_uncertainty ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_chassis 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_chassis.i ${DDS_PATH}/Chassis.cxx ${DDS_PATH}/ChassisPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_chassis ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_control_command 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_control_command.i ${DDS_PATH}/ControlCommand.cxx ${DDS_PATH}/ControlCommandPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_control_command ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_wheel_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_wheel_info.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/WheelInfo.cxx ${DDS_PATH}/WheelInfoPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_wheel_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_control_analysis 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_control_analysis.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/ControlAnalysis.cxx ${DDS_PATH}/ControlAnalysisPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_control_analysis ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_param 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_param.i ${DDS_PATH}/VehicleParam.cxx ${DDS_PATH}/VehicleParamPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_param ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_config 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_config.i ${DDS_PATH}/VehicleParam.cxx ${DDS_PATH}/VehicleParamPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/VehicleConfig.cxx ${DDS_PATH}/VehicleConfigPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_config ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_prediction_obstacles 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_prediction_obstacles.i ${DDS_PATH}/TrajectoryInPrediction.cxx ${DDS_PATH}/TrajectoryInPredictionPubSubTypes.cxx ${DDS_PATH}/ObstacleInteractiveTag.cxx ${DDS_PATH}/ObstacleInteractiveTagPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/ObstacleIntent.cxx ${DDS_PATH}/ObstacleIntentPubSubTypes.cxx ${DDS_PATH}/PredictionObstacles.cxx ${DDS_PATH}/PredictionObstaclesPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/PerceptionObstacle.cxx ${DDS_PATH}/PerceptionObstaclePubSubTypes.cxx ${DDS_PATH}/PredictionObstacle.cxx ${DDS_PATH}/PredictionObstaclePubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/ObstaclePriority.cxx ${DDS_PATH}/ObstaclePriorityPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_prediction_obstacles ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_location 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_location.i ${DDS_PATH}/Location.cxx ${DDS_PATH}/LocationPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PointLLH.cxx ${DDS_PATH}/PointLLHPubSubTypes.cxx ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_location ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_odometry 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_odometry.i ${DDS_PATH}/Quaternion.cxx ${DDS_PATH}/QuaternionPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Odometry.cxx ${DDS_PATH}/OdometryPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_odometry ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle_list.i ${DDS_PATH}/ImageKeyPoint.cxx ${DDS_PATH}/ImageKeyPointPubSubTypes.cxx ${DDS_PATH}/ObstacleList.cxx ${DDS_PATH}/ObstacleListPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/SensorCalibrator.cxx ${DDS_PATH}/SensorCalibratorPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Obstacle.cxx ${DDS_PATH}/ObstaclePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/BBox2D.cxx ${DDS_PATH}/BBox2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lane_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_list.i ${DDS_PATH}/HolisticPathPrediction.cxx ${DDS_PATH}/HolisticPathPredictionPubSubTypes.cxx ${DDS_PATH}/RoadMark.cxx ${DDS_PATH}/RoadMarkPubSubTypes.cxx ${DDS_PATH}/LaneLine.cxx ${DDS_PATH}/LaneLinePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/LaneLineCubicCurve.cxx ${DDS_PATH}/LaneLineCubicCurvePubSubTypes.cxx ${DDS_PATH}/EndPoints.cxx ${DDS_PATH}/EndPointsPubSubTypes.cxx ${DDS_PATH}/LaneList.cxx ${DDS_PATH}/LaneListPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/SensorCalibrator.cxx ${DDS_PATH}/SensorCalibratorPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_traffic_light_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_traffic_light_msg.i ${DDS_PATH}/TrafficLightBox.cxx ${DDS_PATH}/TrafficLightBoxPubSubTypes.cxx ${DDS_PATH}/TrafficLightDebug.cxx ${DDS_PATH}/TrafficLightDebugPubSubTypes.cxx ${DDS_PATH}/ImageRect.cxx ${DDS_PATH}/ImageRectPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/TrafficLightMsg.cxx ${DDS_PATH}/TrafficLightMsgPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/TrafficLight.cxx ${DDS_PATH}/TrafficLightPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_traffic_light_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_ultrasonic 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_ultrasonic.i ${DDS_PATH}/UltrasonicObstacle.cxx ${DDS_PATH}/UltrasonicObstaclePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Ultrasonic.cxx ${DDS_PATH}/UltrasonicPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_ultrasonic ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_radar_obstacle_list_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_radar_obstacle_list_msg.i ${DDS_PATH}/RadarObstacleListMsg.cxx ${DDS_PATH}/RadarObstacleListMsgPubSubTypes.cxx ${DDS_PATH}/RadarState.cxx ${DDS_PATH}/RadarStatePubSubTypes.cxx ${DDS_PATH}/RadarObstacle.cxx ${DDS_PATH}/RadarObstaclePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Status.cxx ${DDS_PATH}/StatusPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/RadarStateError.cxx ${DDS_PATH}/RadarStateErrorPubSubTypes.cxx ${DDS_PATH}/RadarStateMode.cxx ${DDS_PATH}/RadarStateModePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_radar_obstacle_list_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_cloud 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_cloud.i ${DDS_PATH}/PointXYZIRT.cxx ${DDS_PATH}/PointXYZIRTPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PointCloud.cxx ${DDS_PATH}/PointCloudPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_cloud ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_radar_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_radar_obstacle.i ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/RadarObstacle.cxx ${DDS_PATH}/RadarObstaclePubSubTypes.cxx ${DDS_PATH}/Status.cxx ${DDS_PATH}/StatusPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_radar_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_ultrasonic_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_ultrasonic_obstacle.i ${DDS_PATH}/UltrasonicObstacle.cxx ${DDS_PATH}/UltrasonicObstaclePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_ultrasonic_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_xyzirt 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_xyzirt.i ${DDS_PATH}/PointXYZIRT.cxx ${DDS_PATH}/PointXYZIRTPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_xyzirt ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_gnss 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_gnss.i ${DDS_PATH}/Gnss.cxx ${DDS_PATH}/GnssPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_gnss ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_imu 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_imu.i ${DDS_PATH}/Imu.cxx ${DDS_PATH}/ImuPubSubTypes.cxx ${DDS_PATH}/Quaternion.cxx ${DDS_PATH}/QuaternionPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_imu ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_ins 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_ins.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Ins.cxx ${DDS_PATH}/InsPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_ins ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obu_cmd_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obu_cmd_msg.i ${DDS_PATH}/ObuCmdMsg.cxx ${DDS_PATH}/ObuCmdMsgPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/ObuCmd.cxx ${DDS_PATH}/ObuCmdPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obu_cmd_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_routing_request 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_routing_request.i ${DDS_PATH}/KeyPoint.cxx ${DDS_PATH}/KeyPointPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/RoutingRequest.cxx ${DDS_PATH}/RoutingRequestPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_routing_request ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obu_cmd 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obu_cmd.i ${DDS_PATH}/ObuCmd.cxx ${DDS_PATH}/ObuCmdPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obu_cmd ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_info.i ${DDS_PATH}/ParkingStopper.cxx ${DDS_PATH}/ParkingStopperPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ${DDS_PATH}/ParkingInfo.cxx ${DDS_PATH}/ParkingInfoPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_key_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_key_point.i ${DDS_PATH}/KeyPoint.cxx ${DDS_PATH}/KeyPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_key_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_sensor_calibrator 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_sensor_calibrator.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/SensorCalibrator.cxx ${DDS_PATH}/SensorCalibratorPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_sensor_calibrator ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_traffic_light 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_traffic_light.i ${DDS_PATH}/ImageRect.cxx ${DDS_PATH}/ImageRectPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/TrafficLight.cxx ${DDS_PATH}/TrafficLightPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_traffic_light ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_stop_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_stop_point.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/StopPoint.cxx ${DDS_PATH}/StopPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_stop_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lane_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_point.i ${DDS_PATH}/LanePoint.cxx ${DDS_PATH}/LanePointPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lane_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_info.i ${DDS_PATH}/LanePoint.cxx ${DDS_PATH}/LanePointPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/LaneInfo.cxx ${DDS_PATH}/LaneInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle_intent 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle_intent.i ${DDS_PATH}/ObstacleIntent.cxx ${DDS_PATH}/ObstacleIntentPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle_intent ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_routing_response 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_routing_response.i ${DDS_PATH}/LanePoint.cxx ${DDS_PATH}/LanePointPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/RoutingResponse.cxx ${DDS_PATH}/RoutingResponsePubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/LaneInfo.cxx ${DDS_PATH}/LaneInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_routing_response ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_stop_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_stop_info.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/StopInfo.cxx ${DDS_PATH}/StopInfoPubSubTypes.cxx ${DDS_PATH}/StopPoint.cxx ${DDS_PATH}/StopPointPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_stop_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_adc_trajectory 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_adc_trajectory.i ${DDS_PATH}/EStop.cxx ${DDS_PATH}/EStopPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/ADCTrajectory.cxx ${DDS_PATH}/ADCTrajectoryPubSubTypes.cxx ${DDS_PATH}/RSSInfo.cxx ${DDS_PATH}/RSSInfoPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_adc_trajectory ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_bbox_2d 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_bbox_2d.i ${DDS_PATH}/BBox2D.cxx ${DDS_PATH}/BBox2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_bbox_2d ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle.i ${DDS_PATH}/ImageKeyPoint.cxx ${DDS_PATH}/ImageKeyPointPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/SensorCalibrator.cxx ${DDS_PATH}/SensorCalibratorPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Obstacle.cxx ${DDS_PATH}/ObstaclePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/BBox2D.cxx ${DDS_PATH}/BBox2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_rss_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_rss_info.i ${DDS_PATH}/RSSInfo.cxx ${DDS_PATH}/RSSInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_rss_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle_priority 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle_priority.i ${DDS_PATH}/ObstaclePriority.cxx ${DDS_PATH}/ObstaclePriorityPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle_priority ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_planning_cmd 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_planning_cmd.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PlanningCmd.cxx ${DDS_PATH}/PlanningCmdPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_planning_cmd ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_prediction_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_prediction_obstacle.i ${DDS_PATH}/TrajectoryInPrediction.cxx ${DDS_PATH}/TrajectoryInPredictionPubSubTypes.cxx ${DDS_PATH}/ObstacleInteractiveTag.cxx ${DDS_PATH}/ObstacleInteractiveTagPubSubTypes.cxx ${DDS_PATH}/ObstacleIntent.cxx ${DDS_PATH}/ObstacleIntentPubSubTypes.cxx ${DDS_PATH}/PerceptionObstacle.cxx ${DDS_PATH}/PerceptionObstaclePubSubTypes.cxx ${DDS_PATH}/PredictionObstacle.cxx ${DDS_PATH}/PredictionObstaclePubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/ObstaclePriority.cxx ${DDS_PATH}/ObstaclePriorityPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_prediction_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory_in_prediction 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory_in_prediction.i ${DDS_PATH}/TrajectoryInPrediction.cxx ${DDS_PATH}/TrajectoryInPredictionPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory_in_prediction ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory_point_in_prediction 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory_point_in_prediction.i ${DDS_PATH}/TrajectoryPointInPrediction.cxx ${DDS_PATH}/TrajectoryPointInPredictionPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory_point_in_prediction ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_prediction_trajectory_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_prediction_trajectory_point.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/PredictionTrajectoryPoint.cxx ${DDS_PATH}/PredictionTrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_prediction_trajectory_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_perception_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_perception_obstacle.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/PerceptionObstacle.cxx ${DDS_PATH}/PerceptionObstaclePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_perception_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle_feature 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle_feature.i ${DDS_PATH}/TrajectoryInPrediction.cxx ${DDS_PATH}/TrajectoryInPredictionPubSubTypes.cxx ${DDS_PATH}/TrajectoryPointInPrediction.cxx ${DDS_PATH}/TrajectoryPointInPredictionPubSubTypes.cxx ${DDS_PATH}/PredictionTrajectoryPoint.cxx ${DDS_PATH}/PredictionTrajectoryPointPubSubTypes.cxx ${DDS_PATH}/ObstacleFeature.cxx ${DDS_PATH}/ObstacleFeaturePubSubTypes.cxx ${DDS_PATH}/ObstaclePriority.cxx ${DDS_PATH}/ObstaclePriorityPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle_feature ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lane_line 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_line.i ${DDS_PATH}/LaneLine.cxx ${DDS_PATH}/LaneLinePubSubTypes.cxx ${DDS_PATH}/EndPoints.cxx ${DDS_PATH}/EndPointsPubSubTypes.cxx ${DDS_PATH}/LaneLineCubicCurve.cxx ${DDS_PATH}/LaneLineCubicCurvePubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_line ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lane_line_cubic_curve 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lane_line_cubic_curve.i ${DDS_PATH}/LaneLineCubicCurve.cxx ${DDS_PATH}/LaneLineCubicCurvePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lane_line_cubic_curve ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_end_points 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_end_points.i ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/EndPoints.cxx ${DDS_PATH}/EndPointsPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_end_points ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_free_space 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_free_space.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Polygon2D.cxx ${DDS_PATH}/Polygon2DPubSubTypes.cxx ${DDS_PATH}/FreeSpace.cxx ${DDS_PATH}/FreeSpacePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_free_space ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_holistic_path_prediction 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_holistic_path_prediction.i ${DDS_PATH}/HolisticPathPrediction.cxx ${DDS_PATH}/HolisticPathPredictionPubSubTypes.cxx ${DDS_PATH}/LaneLineCubicCurve.cxx ${DDS_PATH}/LaneLineCubicCurvePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_holistic_path_prediction ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_road_mark 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_road_mark.i ${DDS_PATH}/RoadMark.cxx ${DDS_PATH}/RoadMarkPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_road_mark ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_traffic_light_debug 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_traffic_light_debug.i ${DDS_PATH}/TrafficLightBox.cxx ${DDS_PATH}/TrafficLightBoxPubSubTypes.cxx ${DDS_PATH}/TrafficLightDebug.cxx ${DDS_PATH}/TrafficLightDebugPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_traffic_light_debug ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_traffic_light_box 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_traffic_light_box.i ${DDS_PATH}/TrafficLightBox.cxx ${DDS_PATH}/TrafficLightBoxPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_traffic_light_box ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_image_rect 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_image_rect.i ${DDS_PATH}/ImageRect.cxx ${DDS_PATH}/ImageRectPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_image_rect ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_radar_state 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_radar_state.i ${DDS_PATH}/RadarState.cxx ${DDS_PATH}/RadarStatePubSubTypes.cxx ${DDS_PATH}/RadarStateMode.cxx ${DDS_PATH}/RadarStateModePubSubTypes.cxx ${DDS_PATH}/RadarStateError.cxx ${DDS_PATH}/RadarStateErrorPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_radar_state ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_radar_state_error 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_radar_state_error.i ${DDS_PATH}/RadarStateError.cxx ${DDS_PATH}/RadarStateErrorPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_radar_state_error ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_radar_state_mode 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_radar_state_mode.i ${DDS_PATH}/RadarStateMode.cxx ${DDS_PATH}/RadarStateModePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_radar_state_mode ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_sotif_monitor_result 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_sotif_monitor_result.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/SotifMonitorResult.cxx ${DDS_PATH}/SotifMonitorResultPubSubTypes.cxx ${DDS_PATH}/SLPoint.cxx ${DDS_PATH}/SLPointPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Grid.cxx ${DDS_PATH}/GridPubSubTypes.cxx ${DDS_PATH}/Region.cxx ${DDS_PATH}/RegionPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_sotif_monitor_result ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_region 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_region.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Region.cxx ${DDS_PATH}/RegionPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_region ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_grid 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_grid.i ${DDS_PATH}/SLPoint.cxx ${DDS_PATH}/SLPointPubSubTypes.cxx ${DDS_PATH}/Grid.cxx ${DDS_PATH}/GridPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_grid ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_camera_parking_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_camera_parking_info.i ${DDS_PATH}/ImageKeyPoint.cxx ${DDS_PATH}/ImageKeyPointPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/CameraParkingInfo.cxx ${DDS_PATH}/CameraParkingInfoPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_camera_parking_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_camera_parking_info_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_camera_parking_info_list.i ${DDS_PATH}/ImageKeyPoint.cxx ${DDS_PATH}/ImageKeyPointPubSubTypes.cxx ${DDS_PATH}/CameraParkingInfoList.cxx ${DDS_PATH}/CameraParkingInfoListPubSubTypes.cxx ${DDS_PATH}/CameraParkingStopper.cxx ${DDS_PATH}/CameraParkingStopperPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/CameraParkingInfo.cxx ${DDS_PATH}/CameraParkingInfoPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/BBox2D.cxx ${DDS_PATH}/BBox2DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_camera_parking_info_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_security_decision 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_security_decision.i ${DDS_PATH}/SecurityDecision.cxx ${DDS_PATH}/SecurityDecisionPubSubTypes.cxx ${DDS_PATH}/TrajectoryLimitCommand.cxx ${DDS_PATH}/TrajectoryLimitCommandPubSubTypes.cxx ${DDS_PATH}/WarningCommand.cxx ${DDS_PATH}/WarningCommandPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_security_decision ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_warning_command 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_warning_command.i ${DDS_PATH}/WarningCommand.cxx ${DDS_PATH}/WarningCommandPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_warning_command ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory_limit_command 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory_limit_command.i ${DDS_PATH}/TrajectoryLimitCommand.cxx ${DDS_PATH}/TrajectoryLimitCommandPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory_limit_command ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_info_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_info_list.i ${DDS_PATH}/ParkingStopper.cxx ${DDS_PATH}/ParkingStopperPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ${DDS_PATH}/ParkingInfo.cxx ${DDS_PATH}/ParkingInfoPubSubTypes.cxx ${DDS_PATH}/ParkingInfoList.cxx ${DDS_PATH}/ParkingInfoListPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_info_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_comm_command 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_comm_command.i ${DDS_PATH}/Command.cxx ${DDS_PATH}/CommandPubSubTypes.cxx ${DDS_PATH}/CommCommand.cxx ${DDS_PATH}/CommCommandPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_comm_command ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_command_respond 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_command_respond.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/CommandRespond.cxx ${DDS_PATH}/CommandRespondPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_command_respond ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_module_status 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_module_status.i ${DDS_PATH}/ModuleStatus.cxx ${DDS_PATH}/ModuleStatusPubSubTypes.cxx ${DDS_PATH}/Message.cxx ${DDS_PATH}/MessagePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_module_status ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_command 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_command.i ${DDS_PATH}/Command.cxx ${DDS_PATH}/CommandPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_command ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_message 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_message.i ${DDS_PATH}/Message.cxx ${DDS_PATH}/MessagePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_message ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_planning_analysis 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_planning_analysis.i ${DDS_PATH}/Polygon2D.cxx ${DDS_PATH}/Polygon2DPubSubTypes.cxx ${DDS_PATH}/Point2dList.cxx ${DDS_PATH}/Point2dListPubSubTypes.cxx ${DDS_PATH}/Trajectory.cxx ${DDS_PATH}/TrajectoryPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PlanningAnalysis.cxx ${DDS_PATH}/PlanningAnalysisPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/TimeConsume.cxx ${DDS_PATH}/TimeConsumePubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ${DDS_PATH}/PlanningParkingDebug.cxx ${DDS_PATH}/PlanningParkingDebugPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_planning_analysis ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_time_consume 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_time_consume.i ${DDS_PATH}/TimeConsume.cxx ${DDS_PATH}/TimeConsumePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_time_consume ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_planning_parking_debug 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_planning_parking_debug.i ${DDS_PATH}/Point2dList.cxx ${DDS_PATH}/Point2dListPubSubTypes.cxx ${DDS_PATH}/Trajectory.cxx ${DDS_PATH}/TrajectoryPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/Polygon2D.cxx ${DDS_PATH}/Polygon2DPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ${DDS_PATH}/PlanningParkingDebug.cxx ${DDS_PATH}/PlanningParkingDebugPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_planning_parking_debug ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_trajectory_array 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_trajectory_array.i ${DDS_PATH}/Trajectory.cxx ${DDS_PATH}/TrajectoryPubSubTypes.cxx ${DDS_PATH}/TrajectoryArray.cxx ${DDS_PATH}/TrajectoryArrayPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/PathPoint.cxx ${DDS_PATH}/PathPointPubSubTypes.cxx ${DDS_PATH}/Path.cxx ${DDS_PATH}/PathPubSubTypes.cxx ${DDS_PATH}/TrajectoryPoint.cxx ${DDS_PATH}/TrajectoryPointPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_trajectory_array ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_vehicle_state 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_vehicle_state.i ${DDS_PATH}/VehicleState.cxx ${DDS_PATH}/VehicleStatePubSubTypes.cxx ${DDS_PATH}/Quaternion.cxx ${DDS_PATH}/QuaternionPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Pose.cxx ${DDS_PATH}/PosePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_vehicle_state ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_drivable_region 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_drivable_region.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/DrivableRegion.cxx ${DDS_PATH}/DrivableRegionPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_drivable_region ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_obstacle_interactive_tag 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_obstacle_interactive_tag.i ${DDS_PATH}/ObstacleInteractiveTag.cxx ${DDS_PATH}/ObstacleInteractiveTagPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_obstacle_interactive_tag ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_twist 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_twist.i ${DDS_PATH}/Twist.cxx ${DDS_PATH}/TwistPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_twist ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_state_display 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_state_display.i ${DDS_PATH}/ParkingStateDisplay.cxx ${DDS_PATH}/ParkingStateDisplayPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_state_display ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_roi 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_roi.i ${DDS_PATH}/ParkingRoi.cxx ${DDS_PATH}/ParkingRoiPubSubTypes.cxx ${DDS_PATH}/Point2dList.cxx ${DDS_PATH}/Point2dListPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_roi ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_point_2d_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_point_2d_list.i ${DDS_PATH}/Point2D.cxx ${DDS_PATH}/Point2DPubSubTypes.cxx ${DDS_PATH}/Point2dList.cxx ${DDS_PATH}/Point2dListPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_point_2d_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_out_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_out_info.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/ParkingOutInfo.cxx ${DDS_PATH}/ParkingOutInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_out_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_junction_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_junction_info.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/JunctionInfo.cxx ${DDS_PATH}/JunctionInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_junction_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_traffic_events 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_traffic_events.i ${DDS_PATH}/RouteFusionInfo.cxx ${DDS_PATH}/RouteFusionInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/LimitSpeedInfo.cxx ${DDS_PATH}/LimitSpeedInfoPubSubTypes.cxx ${DDS_PATH}/TrafficEvents.cxx ${DDS_PATH}/TrafficEventsPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/JunctionInfo.cxx ${DDS_PATH}/JunctionInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_traffic_events ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_limit_speed_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_limit_speed_info.i ${DDS_PATH}/LimitSpeedInfo.cxx ${DDS_PATH}/LimitSpeedInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_limit_speed_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_log 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_log.i ${DDS_PATH}/Log.cxx ${DDS_PATH}/LogPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_log ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_pavementype 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_pavementype.i ${DDS_PATH}/Pavementype.cxx ${DDS_PATH}/PavementypePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_pavementype ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_wl_constraint_info_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_wl_constraint_info_list.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/WLConstraintInfo.cxx ${DDS_PATH}/WLConstraintInfoPubSubTypes.cxx ${DDS_PATH}/WLConstraintInfoList.cxx ${DDS_PATH}/WLConstraintInfoListPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_wl_constraint_info_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_wl_constraint_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_wl_constraint_info.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/WLConstraintInfo.cxx ${DDS_PATH}/WLConstraintInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_wl_constraint_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_uss_obstacle_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_uss_obstacle_list.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/UssObstacle.cxx ${DDS_PATH}/UssObstaclePubSubTypes.cxx ${DDS_PATH}/UssObstacleList.cxx ${DDS_PATH}/UssObstacleListPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_uss_obstacle_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_uss_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_uss_obstacle.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/UssObstacle.cxx ${DDS_PATH}/UssObstaclePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_uss_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_uss_parking_info_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_uss_parking_info_list.i ${DDS_PATH}/UssParkingInfo.cxx ${DDS_PATH}/UssParkingInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/UssParkingInfoList.cxx ${DDS_PATH}/UssParkingInfoListPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_uss_parking_info_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_uss_parking_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_uss_parking_info.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/UssParkingInfo.cxx ${DDS_PATH}/UssParkingInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_uss_parking_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_camera_parking_stopper 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_camera_parking_stopper.i ${DDS_PATH}/CameraParkingStopper.cxx ${DDS_PATH}/CameraParkingStopperPubSubTypes.cxx ${DDS_PATH}/BBox2D.cxx ${DDS_PATH}/BBox2DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_camera_parking_stopper ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_parking_stopper 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_parking_stopper.i ${DDS_PATH}/ParkingStopper.cxx ${DDS_PATH}/ParkingStopperPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_parking_stopper ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_diagnostic_array 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_diagnostic_array.i ${DDS_PATH}/DiagnosticStatus.cxx ${DDS_PATH}/DiagnosticStatusPubSubTypes.cxx ${DDS_PATH}/KeyValues.cxx ${DDS_PATH}/KeyValuesPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/DiagnosticArray.cxx ${DDS_PATH}/DiagnosticArrayPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_diagnostic_array ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_diagnostic_status 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_diagnostic_status.i ${DDS_PATH}/KeyValues.cxx ${DDS_PATH}/KeyValuesPubSubTypes.cxx ${DDS_PATH}/DiagnosticStatus.cxx ${DDS_PATH}/DiagnosticStatusPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_diagnostic_status ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_key_values 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_key_values.i ${DDS_PATH}/KeyValues.cxx ${DDS_PATH}/KeyValuesPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_key_values ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_alarm_message 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_alarm_message.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/AlarmMessage.cxx ${DDS_PATH}/AlarmMessagePubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_alarm_message ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_lanelet_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_lanelet_info.i ${DDS_PATH}/LaneletInfo.cxx ${DDS_PATH}/LaneletInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_lanelet_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_global_route_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_global_route_msg.i ${DDS_PATH}/GlobalRouteMsg.cxx ${DDS_PATH}/GlobalRouteMsgPubSubTypes.cxx ${DDS_PATH}/LaneletInfo.cxx ${DDS_PATH}/LaneletInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_global_route_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_route_fusion_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_route_fusion_info.i ${DDS_PATH}/RouteFusionInfo.cxx ${DDS_PATH}/RouteFusionInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_route_fusion_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_road_mark_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_road_mark_list.i ${DDS_PATH}/RoadMark.cxx ${DDS_PATH}/RoadMarkPubSubTypes.cxx ${DDS_PATH}/RoadMarkList.cxx ${DDS_PATH}/RoadMarkListPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_road_mark_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_obu_cmd 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_obu_cmd.i ${DDS_PATH}/HMIObuCmd.cxx ${DDS_PATH}/HMIObuCmdPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_obu_cmd ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_obu_cmd_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_obu_cmd_msg.i ${DDS_PATH}/HMIObuCmd.cxx ${DDS_PATH}/HMIObuCmdPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/HMIObuCmdMsg.cxx ${DDS_PATH}/HMIObuCmdMsgPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_obu_cmd_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_parking_state_display 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_parking_state_display.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/HMIParkingStateDisplay.cxx ${DDS_PATH}/HMIParkingStateDisplayPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_parking_state_display ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_parking_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_parking_info.i ${DDS_PATH}/HMIParkingInfo.cxx ${DDS_PATH}/HMIParkingInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_parking_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_parking_info_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_parking_info_list.i ${DDS_PATH}/HMIParkingInfo.cxx ${DDS_PATH}/HMIParkingInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Polygon3D.cxx ${DDS_PATH}/Polygon3DPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/HMIParkingInfoList.cxx ${DDS_PATH}/HMIParkingInfoListPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_parking_info_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_vehicle_msg 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_vehicle_msg.i ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/PointLLH.cxx ${DDS_PATH}/PointLLHPubSubTypes.cxx ${DDS_PATH}/PointENU.cxx ${DDS_PATH}/PointENUPubSubTypes.cxx ${DDS_PATH}/HMIVehicleMsg.cxx ${DDS_PATH}/HMIVehicleMsgPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_vehicle_msg ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_obstacle 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_obstacle.i ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ${DDS_PATH}/HMIObstacle.cxx ${DDS_PATH}/HMIObstaclePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_obstacle ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_obstacle_list 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_obstacle_list.i ${DDS_PATH}/HMIObstacleList.cxx ${DDS_PATH}/HMIObstacleListPubSubTypes.cxx ${DDS_PATH}/HMIObstacle.cxx ${DDS_PATH}/HMIObstaclePubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_obstacle_list ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_trajectory_point 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_trajectory_point.i ${DDS_PATH}/HMITrajectoryPoint.cxx ${DDS_PATH}/HMITrajectoryPointPubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_trajectory_point ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_trajectory 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_trajectory.i ${DDS_PATH}/HMITrajectoryPoint.cxx ${DDS_PATH}/HMITrajectoryPointPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/HMITrajectory.cxx ${DDS_PATH}/HMITrajectoryPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/Point3D.cxx ${DDS_PATH}/Point3DPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_trajectory ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_diagnostic_status 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_diagnostic_status.i ${DDS_PATH}/HMIDiagnosticStatus.cxx ${DDS_PATH}/HMIDiagnosticStatusPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_diagnostic_status ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_hmi_diagnostic_array 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_hmi_diagnostic_array.i ${DDS_PATH}/HMIDiagnosticStatus.cxx ${DDS_PATH}/HMIDiagnosticStatusPubSubTypes.cxx ${DDS_PATH}/HMIDiagnosticArray.cxx ${DDS_PATH}/HMIDiagnosticArrayPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_hmi_diagnostic_array ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_guide_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_guide_info.i ${DDS_PATH}/CurvatureInfo.cxx ${DDS_PATH}/CurvatureInfoPubSubTypes.cxx ${DDS_PATH}/Header.cxx ${DDS_PATH}/HeaderPubSubTypes.cxx ${DDS_PATH}/GuideInfo.cxx ${DDS_PATH}/GuideInfoPubSubTypes.cxx ${DDS_PATH}/Time.cxx ${DDS_PATH}/TimePubSubTypes.cxx ${DDS_PATH}/GuideRoad.cxx ${DDS_PATH}/GuideRoadPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_guide_info ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_guide_road 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_guide_road.i ${DDS_PATH}/CurvatureInfo.cxx ${DDS_PATH}/CurvatureInfoPubSubTypes.cxx ${DDS_PATH}/GuideRoad.cxx ${DDS_PATH}/GuideRoadPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_guide_road ${PYTHON_LIBRARIES} fastrtps) 

SWIG_ADD_LIBRARY(fastdds_curvature_info 
    TYPE SHARED 
    LANGUAGE python 
    SOURCES ${SWIG_SOURCE}/fastdds_curvature_info.i ${DDS_PATH}/CurvatureInfo.cxx ${DDS_PATH}/CurvatureInfoPubSubTypes.cxx ) 
SWIG_LINK_LIBRARIES(fastdds_curvature_info ${PYTHON_LIBRARIES} fastrtps) 

############################################################################### 
# Copy python files 
############################################################################### 

configure_file(${PROJECT_SOURCE_DIR}/dds_message.py 
    ${CMAKE_CURRENT_BINARY_DIR}/dds_message.py COPYONLY) 
