/**
 * @file    ros_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "ros_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"

#if ROS_ENABLE
/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
using namespace legionclaw::common;
template <typename T> void RosMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  action_mode_ = MessageActionMode::DO_NOTHING;

  routing_response_pub_ = nh_.advertise<::ros_interface::RoutingResponse>(
      "/routing/RoutingResponse", 10);


  guide_info_pub_ = nh_.advertise<::ros_interface::GuideInfo>("/routing/GuideInfo", 10);

  lane_list_pub_ = nh_.advertise<::ros_interface::LaneList>("/routing/LaneList", 10);
  
  parking_info_pub_ =
      nh_.advertise<::ros_interface::ParkingInfo>("/routing/ParkingInfo", 10);

  stop_info_pub_ =
      nh_.advertise<::ros_interface::StopInfo>("/routing/StopInfo", 10);

  global_route_msg_pub_ = nh_.advertise<::ros_interface::GlobalRouteMsg>(
      "/routing/GlobalRouteMsg", 10);

  faults_pub_ = nh_.advertise<::ros_interface::Faults>("/routing/Faults", 10);

  polygon_2d_pub_ =
      nh_.advertise<::ros_interface::Polygon2D>("/routing/MapBoundary", 10);

  traffic_events_output_pub_ = nh_.advertise<::ros_interface::TrafficEvents>(
      "/routing/TrafficEvents", 10);

  obu_cmd_msg_sub_ =
      nh_.subscribe("/vui_client/ObuCmdMsg", 30,
                    &RosMessageManager::HandleObuCmdMsgMessage, this);
  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  register_thread_.reset(new std::thread([this] { RegisterLoop(); }));
  if (register_thread_ == nullptr) {
    AERROR << "Unable to create register_thread_ thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool RosMessageManager<T>::Activate() {
  std::unique_lock<std::mutex> lock(mode_mutex_);
  if (is_active_ == true) {
    std::cout << "already start" << std::endl;
    action_mode_ = MessageActionMode::DO_NOTHING;
    return false;
  } else {
    action_mode_ = MessageActionMode::TO_ACTIVATE;
  }
  return true;
}

template <typename T> void RosMessageManager<T>::TaskStart() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_active_ == true) {
    return;
  }
  traffic_light_msg_sub_ =
      nh_.subscribe("/perception/fusion/traffic_sign_fusion/TrafficLightMsg",
                    30, &RosMessageManager::HandleTrafficLightMsgMessage, this);

  location_sub_ =
      nh_.subscribe("/localization/global_fusion/Location", 30,
                    &RosMessageManager::HandleLocationMessage, this);

  chassis_sub_ = nh_.subscribe("/drivers/canbus/Chassis", 30,
                               &RosMessageManager::HandleChassisMessage, this);

  traffic_events_input_sub_ =
      nh_.subscribe("/vui_client/SpeedLimit", 30,
                    &RosMessageManager::HandleTrafficEventsInputMessage, this);

  odometry_sub_ =
      nh_.subscribe("/localization/local_map_location/Map2LocalTF", 30,
                    &RosMessageManager::HandleOdometryMessage, this);

  routing_request_sub_ =
      nh_.subscribe("/vui_client/RoutingRequest", 30,
                    &RosMessageManager::HandleRoutingRequestMessage, this);

  std::cout << "ros1 activate" << std::endl;
  is_active_ = true;
  action_mode_ = MessageActionMode::DO_NOTHING;
  return;
}
template <typename T> bool RosMessageManager<T>::DeActivate() {
  std::unique_lock<std::mutex> lock(mode_mutex_);
  if (is_active_ == false) {
    std::cout << "already stop" << std::endl;
    action_mode_ = MessageActionMode::DO_NOTHING;
    return false;
  } else {
    action_mode_ = MessageActionMode::TO_DEACTIVATE;
  }
  return true;
}

template <typename T> void RosMessageManager<T>::TaskStop() {
  if (is_active_ == false) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  traffic_light_msg_sub_.shutdown();

  location_sub_.shutdown();

  chassis_sub_.shutdown();

  traffic_events_input_sub_.shutdown();

  odometry_sub_.shutdown();

  routing_request_sub_.shutdown();

  std::cout << "ros1 deactivate" << std::endl;
  is_active_ = false;
  action_mode_ = MessageActionMode::DO_NOTHING;
  return;
}
template <typename T>
void RosMessageManager<T>::PublishRoutingResponse(
    legionclaw::interface::RoutingResponse msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::RoutingResponse routing_response;
  MESSAGE_HEADER_ASSIGN(std_msgs, routing_response)
  routing_response.plan_status = msg.plan_status();
  routing_response.replan_flag = msg.replan_flag();
  routing_response.route_reason = msg.route_reason();
  std::vector<ros_interface::LaneInfo> ros_lane_list;
  std::vector<legionclaw::interface::LaneInfo> legion_lane_list;
  msg.lane_list(legion_lane_list);
  for (auto it_lane_list : legion_lane_list) {
    ::ros_interface::LaneInfo routing_response_lane_info;
    routing_response_lane_info.priority = it_lane_list.priority();
    routing_response_lane_info.global_id = it_lane_list.global_id();
    routing_response_lane_info.predecessor_id = it_lane_list.predecessor_id();
    routing_response_lane_info.successor_id = it_lane_list.successor_id();
    routing_response_lane_info.left_neighbor_id = it_lane_list.left_neighbor_id();
    routing_response_lane_info.right_neighbor_id = it_lane_list.right_neighbor_id();
    routing_response_lane_info.type = it_lane_list.type();
    std::vector<ros_interface::LanePoint> ros_lane_points;
    std::vector<legionclaw::interface::LanePoint> legion_lane_points;
    it_lane_list.lane_points(legion_lane_points);
    for (auto it_lane_points : legion_lane_points) {
      ::ros_interface::LanePoint routing_response_lane_info_lane_point;
      ::ros_interface::Point3D routing_response_lane_info_lane_point_point;
      routing_response_lane_info_lane_point_point.x =
          it_lane_points.point().x();
      routing_response_lane_info_lane_point_point.y =
          it_lane_points.point().y();
      routing_response_lane_info_lane_point_point.z =
          it_lane_points.point().z();
      routing_response_lane_info_lane_point.point =
          routing_response_lane_info_lane_point_point;
      routing_response_lane_info_lane_point.theta = it_lane_points.theta();
      routing_response_lane_info_lane_point.mileage = it_lane_points.mileage();
      routing_response_lane_info_lane_point.limit_speed =
          it_lane_points.limit_speed();
      routing_response_lane_info_lane_point.left_road_width =
          it_lane_points.left_road_width();
      routing_response_lane_info_lane_point.right_road_width =
          it_lane_points.right_road_width();
      routing_response_lane_info_lane_point.left_line_type =
          it_lane_points.left_line_type();
      routing_response_lane_info_lane_point.right_line_type =
          it_lane_points.right_line_type();
      ros_lane_points.emplace_back(routing_response_lane_info_lane_point);
    }
    routing_response_lane_info.lane_points = ros_lane_points;
    ros_lane_list.emplace_back(routing_response_lane_info);
  }
  routing_response.lane_list = ros_lane_list;

  routing_response_pub_.publish(routing_response);
}

template <typename T>
void RosMessageManager<T>::PublishGuideInfo(legionclaw::interface::GuideInfo msg) {
  if (is_init_ == false) return;
  ::ros_interface::GuideInfo guide_info;
  MESSAGE_HEADER_ASSIGN(std_msgs,guide_info)
  guide_info.next_dis = msg.next_dis();
  ::ros_interface::GuideRoad guide_info_current_road;
  guide_info_current_road.road_id = msg.current_road().road_id();
  guide_info_current_road.road_type = msg.current_road().road_type();
  guide_info_current_road.turn_type = msg.current_road().turn_type();
  guide_info_current_road.avg_curvature = msg.current_road().avg_curvature();
  guide_info_current_road.curvature_size = msg.current_road().curvature_size();
    std::vector<ros_interface::CurvatureInfo> ros_curvature;
  std::vector<legionclaw::interface::CurvatureInfo> legion_curvature;
  msg.current_road().curvature(legion_curvature);
  for (auto it_curvature : legion_curvature) {
  ::ros_interface::CurvatureInfo guide_info_current_road_curvature_info;
  guide_info_current_road_curvature_info.offset = it_curvature.offset();
  guide_info_current_road_curvature_info.value = it_curvature.value();
ros_curvature.emplace_back(guide_info_current_road_curvature_info);
  }
  guide_info_current_road.curvature = ros_curvature;
guide_info.current_road = guide_info_current_road;
  ::ros_interface::GuideRoad guide_info_next_road;
  guide_info_next_road.road_id = msg.next_road().road_id();
  guide_info_next_road.road_type = msg.next_road().road_type();
  guide_info_next_road.turn_type = msg.next_road().turn_type();
  guide_info_next_road.avg_curvature = msg.next_road().avg_curvature();
  guide_info_next_road.curvature_size = msg.next_road().curvature_size();
    std::vector<ros_interface::CurvatureInfo> ros_curvature_1;
  std::vector<legionclaw::interface::CurvatureInfo> legion_curvature_1;
  msg.next_road().curvature(legion_curvature_1);
  for (auto it_curvature_1 : legion_curvature_1) {
  ::ros_interface::CurvatureInfo guide_info_next_road_curvature_info;
  guide_info_next_road_curvature_info.offset = it_curvature_1.offset();
  guide_info_next_road_curvature_info.value = it_curvature_1.value();
ros_curvature_1.emplace_back(guide_info_next_road_curvature_info);
  }
  guide_info_next_road.curvature = ros_curvature_1;
guide_info.next_road = guide_info_next_road;
  guide_info.round_status = msg.round_status();
  guide_info.intersection_status = msg.intersection_status();
  guide_info.roads_status = msg.roads_status();

  guide_info_pub_.publish(guide_info);
}

template <typename T>
void RosMessageManager<T>::PublishLaneList(legionclaw::interface::LaneList msg) {
  if (is_init_ == false) return;
  ::ros_interface::LaneList lane_list;
  MESSAGE_HEADER_ASSIGN(std_msgs,lane_list)
  lane_list.sensor_id = msg.sensor_id();
  lane_list.error_code = msg.error_code();
  lane_list.sensor_status = msg.sensor_status();
  lane_list.change_origin_flag = msg.change_origin_flag();
  lane_list.is_valid = msg.is_valid();
  ::ros_interface::SensorCalibrator lane_list_sensor_calibrator;
  ::ros_interface::Point3D lane_list_sensor_calibrator_pose;
  lane_list_sensor_calibrator_pose.x = msg.sensor_calibrator().pose().x();
  lane_list_sensor_calibrator_pose.y = msg.sensor_calibrator().pose().y();
  lane_list_sensor_calibrator_pose.z = msg.sensor_calibrator().pose().z();
lane_list_sensor_calibrator.pose = lane_list_sensor_calibrator_pose;
  ::ros_interface::Point3D lane_list_sensor_calibrator_angle;
  lane_list_sensor_calibrator_angle.x = msg.sensor_calibrator().angle().x();
  lane_list_sensor_calibrator_angle.y = msg.sensor_calibrator().angle().y();
  lane_list_sensor_calibrator_angle.z = msg.sensor_calibrator().angle().z();
lane_list_sensor_calibrator.angle = lane_list_sensor_calibrator_angle;
lane_list.sensor_calibrator = lane_list_sensor_calibrator;
    std::vector<ros_interface::LaneLine> ros_camera_laneline;
  std::vector<legionclaw::interface::LaneLine> legion_camera_laneline;
  msg.camera_laneline(legion_camera_laneline);
  for (auto it_camera_laneline : legion_camera_laneline) {
  ::ros_interface::LaneLine lane_list_lane_line;
  lane_list_lane_line.lane_type = it_camera_laneline.lane_type();
  lane_list_lane_line.lane_color = it_camera_laneline.lane_color();
  lane_list_lane_line.pos_type = it_camera_laneline.pos_type();
  ::ros_interface::LaneLineCubicCurve lane_list_lane_line_curve_vehicle;
  lane_list_lane_line_curve_vehicle.start_x = it_camera_laneline.curve_vehicle().start_x();
  lane_list_lane_line_curve_vehicle.end_x = it_camera_laneline.curve_vehicle().end_x();
  lane_list_lane_line_curve_vehicle.a = it_camera_laneline.curve_vehicle().a();
  lane_list_lane_line_curve_vehicle.b = it_camera_laneline.curve_vehicle().b();
  lane_list_lane_line_curve_vehicle.c = it_camera_laneline.curve_vehicle().c();
  lane_list_lane_line_curve_vehicle.d = it_camera_laneline.curve_vehicle().d();
lane_list_lane_line.curve_vehicle = lane_list_lane_line_curve_vehicle;
  ::ros_interface::LaneLineCubicCurve lane_list_lane_line_curve_image;
  lane_list_lane_line_curve_image.start_x = it_camera_laneline.curve_image().start_x();
  lane_list_lane_line_curve_image.end_x = it_camera_laneline.curve_image().end_x();
  lane_list_lane_line_curve_image.a = it_camera_laneline.curve_image().a();
  lane_list_lane_line_curve_image.b = it_camera_laneline.curve_image().b();
  lane_list_lane_line_curve_image.c = it_camera_laneline.curve_image().c();
  lane_list_lane_line_curve_image.d = it_camera_laneline.curve_image().d();
lane_list_lane_line.curve_image = lane_list_lane_line_curve_image;
  ::ros_interface::LaneLineCubicCurve lane_list_lane_line_curve_abs;
  lane_list_lane_line_curve_abs.start_x = it_camera_laneline.curve_abs().start_x();
  lane_list_lane_line_curve_abs.end_x = it_camera_laneline.curve_abs().end_x();
  lane_list_lane_line_curve_abs.a = it_camera_laneline.curve_abs().a();
  lane_list_lane_line_curve_abs.b = it_camera_laneline.curve_abs().b();
  lane_list_lane_line_curve_abs.c = it_camera_laneline.curve_abs().c();
  lane_list_lane_line_curve_abs.d = it_camera_laneline.curve_abs().d();
lane_list_lane_line.curve_abs = lane_list_lane_line_curve_abs;
    std::vector<ros_interface::Point3D> ros_pts_vehicle;
  std::vector<legionclaw::interface::Point3D> legion_pts_vehicle;
  it_camera_laneline.pts_vehicle(legion_pts_vehicle);
  for (auto it_pts_vehicle : legion_pts_vehicle) {
  ::ros_interface::Point3D lane_list_lane_line_point_3d;
  lane_list_lane_line_point_3d.x = it_pts_vehicle.x();
  lane_list_lane_line_point_3d.y = it_pts_vehicle.y();
  lane_list_lane_line_point_3d.z = it_pts_vehicle.z();
ros_pts_vehicle.emplace_back(lane_list_lane_line_point_3d);
  }
  lane_list_lane_line.pts_vehicle = ros_pts_vehicle;
    std::vector<ros_interface::Point2D> ros_pts_image;
  std::vector<legionclaw::interface::Point2D> legion_pts_image;
  it_camera_laneline.pts_image(legion_pts_image);
  for (auto it_pts_image : legion_pts_image) {
  ::ros_interface::Point2D lane_list_lane_line_point_2d;
  lane_list_lane_line_point_2d.x = it_pts_image.x();
  lane_list_lane_line_point_2d.y = it_pts_image.y();
ros_pts_image.emplace_back(lane_list_lane_line_point_2d);
  }
  lane_list_lane_line.pts_image = ros_pts_image;
    std::vector<ros_interface::Point3D> ros_pts_abs;
  std::vector<legionclaw::interface::Point3D> legion_pts_abs;
  it_camera_laneline.pts_abs(legion_pts_abs);
  for (auto it_pts_abs : legion_pts_abs) {
  ::ros_interface::Point3D lane_list_lane_line_point_3d;
  lane_list_lane_line_point_3d.x = it_pts_abs.x();
  lane_list_lane_line_point_3d.y = it_pts_abs.y();
  lane_list_lane_line_point_3d.z = it_pts_abs.z();
ros_pts_abs.emplace_back(lane_list_lane_line_point_3d);
  }
  lane_list_lane_line.pts_abs = ros_pts_abs;
  ::ros_interface::EndPoints lane_list_lane_line_image_end_point;
  ::ros_interface::Point2D lane_list_lane_line_image_end_point_start;
  lane_list_lane_line_image_end_point_start.x = it_camera_laneline.image_end_point().start().x();
  lane_list_lane_line_image_end_point_start.y = it_camera_laneline.image_end_point().start().y();
lane_list_lane_line_image_end_point.start = lane_list_lane_line_image_end_point_start;
  ::ros_interface::Point2D lane_list_lane_line_image_end_point_end;
  lane_list_lane_line_image_end_point_end.x = it_camera_laneline.image_end_point().end().x();
  lane_list_lane_line_image_end_point_end.y = it_camera_laneline.image_end_point().end().y();
lane_list_lane_line_image_end_point.end = lane_list_lane_line_image_end_point_end;
lane_list_lane_line.image_end_point = lane_list_lane_line_image_end_point;
    std::vector<ros_interface::Point2D> ros_pts_key;
  std::vector<legionclaw::interface::Point2D> legion_pts_key;
  it_camera_laneline.pts_key(legion_pts_key);
  for (auto it_pts_key : legion_pts_key) {
  ::ros_interface::Point2D lane_list_lane_line_point_2d;
  lane_list_lane_line_point_2d.x = it_pts_key.x();
  lane_list_lane_line_point_2d.y = it_pts_key.y();
ros_pts_key.emplace_back(lane_list_lane_line_point_2d);
  }
  lane_list_lane_line.pts_key = ros_pts_key;
  lane_list_lane_line.hd_lane_id = it_camera_laneline.hd_lane_id();
  lane_list_lane_line.confidence = it_camera_laneline.confidence();
  lane_list_lane_line.lane_quality = it_camera_laneline.lane_quality();
  lane_list_lane_line.fused_lane_type = it_camera_laneline.fused_lane_type();
  std::vector<double> homography_mat;
  it_camera_laneline.homography_mat(homography_mat);
  lane_list_lane_line.homography_mat = homography_mat;
  std::vector<double> homography_mat_inv;
  it_camera_laneline.homography_mat_inv(homography_mat_inv);
  lane_list_lane_line.homography_mat_inv = homography_mat_inv;
  lane_list_lane_line.lane_coordinate_type = it_camera_laneline.lane_coordinate_type();
  lane_list_lane_line.use_type = it_camera_laneline.use_type();
  ::ros_interface::Time lane_list_lane_line_create_time;
  lane_list_lane_line_create_time.sec = it_camera_laneline.create_time().sec();
  lane_list_lane_line_create_time.nsec = it_camera_laneline.create_time().nsec();
lane_list_lane_line.create_time = lane_list_lane_line_create_time;
ros_camera_laneline.emplace_back(lane_list_lane_line);
  }
  lane_list.camera_laneline = ros_camera_laneline;
  ::ros_interface::HolisticPathPrediction lane_list_hpp;
  ::ros_interface::LaneLineCubicCurve lane_list_hpp_hpp;
  lane_list_hpp_hpp.start_x = msg.hpp().hpp().start_x();
  lane_list_hpp_hpp.end_x = msg.hpp().hpp().end_x();
  lane_list_hpp_hpp.a = msg.hpp().hpp().a();
  lane_list_hpp_hpp.b = msg.hpp().hpp().b();
  lane_list_hpp_hpp.c = msg.hpp().hpp().c();
  lane_list_hpp_hpp.d = msg.hpp().hpp().d();
lane_list_hpp.hpp = lane_list_hpp_hpp;
  lane_list_hpp.planning_source = msg.hpp().planning_source();
  lane_list_hpp.ego_lane_width = msg.hpp().ego_lane_width();
  lane_list_hpp.confidence = msg.hpp().confidence();
lane_list.hpp = lane_list_hpp;
    std::vector<ros_interface::RoadMark> ros_road_marks;
  std::vector<legionclaw::interface::RoadMark> legion_road_marks;
  msg.road_marks(legion_road_marks);
  for (auto it_road_marks : legion_road_marks) {
  ::ros_interface::RoadMark lane_list_road_mark;
  lane_list_road_mark.longitude_dist = it_road_marks.longitude_dist();
  lane_list_road_mark.lateral_dist = it_road_marks.lateral_dist();
  lane_list_road_mark.x = it_road_marks.x();
  lane_list_road_mark.y = it_road_marks.y();
  lane_list_road_mark.z = it_road_marks.z();
  lane_list_road_mark.confidence = it_road_marks.confidence();
  lane_list_road_mark.type = it_road_marks.type();
ros_road_marks.emplace_back(lane_list_road_mark);
  }
  lane_list.road_marks = ros_road_marks;

  lane_list_pub_.publish(lane_list);
}


template <typename T>
void RosMessageManager<T>::PublishParkingInfo(
    legionclaw::interface::ParkingInfo msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::ParkingInfo parking_info;
  MESSAGE_HEADER_ASSIGN(std_msgs, parking_info)
  parking_info.parking_space_id = msg.parking_space_id();
  parking_info.parking_type = msg.parking_type();
  parking_info.parking_status = msg.parking_status();
  parking_info.confidence = msg.confidence();
  ::ros_interface::Point3D parking_info_center_point_of_parking;
  parking_info_center_point_of_parking.x = msg.center_point_of_parking().x();
  parking_info_center_point_of_parking.y = msg.center_point_of_parking().y();
  parking_info_center_point_of_parking.z = msg.center_point_of_parking().z();
  parking_info.center_point_of_parking = parking_info_center_point_of_parking;
  parking_info.theta = msg.theta();
  parking_info.width = msg.width();
  parking_info.length = msg.length();
  parking_info.yaw_offset = msg.yaw_offset();
  ::ros_interface::Polygon3D parking_info_polygon;
  parking_info_polygon.coordinate_system = msg.polygon().coordinate_system();
  std::vector<ros_interface::Point3D> ros_points;
  std::vector<legionclaw::interface::Point3D> legion_points;
  msg.polygon().points(legion_points);
  for (auto it_points : legion_points) {
    ::ros_interface::Point3D parking_info_polygon_point_3d;
    parking_info_polygon_point_3d.x = it_points.x();
    parking_info_polygon_point_3d.y = it_points.y();
    parking_info_polygon_point_3d.z = it_points.z();
    ros_points.emplace_back(parking_info_polygon_point_3d);
  }
  parking_info_polygon.points = ros_points;
  parking_info.polygon = parking_info_polygon;
  parking_info.sensor_id = msg.sensor_id();
  parking_info.is_lane_width_valid = msg.is_lane_width_valid();
  parking_info.lane_width = msg.lane_width();
  std::vector<ros_interface::ParkingStopper> ros_parking_stoppers;
  std::vector<legionclaw::interface::ParkingStopper> legion_parking_stoppers;
  msg.parking_stoppers(legion_parking_stoppers);
  for (auto it_parking_stoppers : legion_parking_stoppers) {
    ::ros_interface::ParkingStopper parking_info_parking_stopper;
    MESSAGE_HEADER_ASSIGN(std_msgs, parking_info_parking_stopper)
    ::ros_interface::Point3D parking_info_parking_stopper_center_point_vehicle;
    parking_info_parking_stopper_center_point_vehicle.x =
        it_parking_stoppers.center_point_vehicle().x();
    parking_info_parking_stopper_center_point_vehicle.y =
        it_parking_stoppers.center_point_vehicle().y();
    parking_info_parking_stopper_center_point_vehicle.z =
        it_parking_stoppers.center_point_vehicle().z();
    parking_info_parking_stopper.center_point_vehicle =
        parking_info_parking_stopper_center_point_vehicle;
    ::ros_interface::Point3D parking_info_parking_stopper_center_point_abs;
    parking_info_parking_stopper_center_point_abs.x =
        it_parking_stoppers.center_point_abs().x();
    parking_info_parking_stopper_center_point_abs.y =
        it_parking_stoppers.center_point_abs().y();
    parking_info_parking_stopper_center_point_abs.z =
        it_parking_stoppers.center_point_abs().z();
    parking_info_parking_stopper.center_point_abs =
        parking_info_parking_stopper_center_point_abs;
    std::vector<ros_interface::Point3D> ros_stopper_points_vehicle;
    std::vector<legionclaw::interface::Point3D> legion_stopper_points_vehicle;
    it_parking_stoppers.stopper_points_vehicle(legion_stopper_points_vehicle);
    for (auto it_stopper_points_vehicle : legion_stopper_points_vehicle) {
      ::ros_interface::Point3D parking_info_parking_stopper_point_3d;
      parking_info_parking_stopper_point_3d.x = it_stopper_points_vehicle.x();
      parking_info_parking_stopper_point_3d.y = it_stopper_points_vehicle.y();
      parking_info_parking_stopper_point_3d.z = it_stopper_points_vehicle.z();
      ros_stopper_points_vehicle.emplace_back(
          parking_info_parking_stopper_point_3d);
    }
    parking_info_parking_stopper.stopper_points_vehicle =
        ros_stopper_points_vehicle;
    std::vector<ros_interface::Point3D> ros_stopper_points_abs;
    std::vector<legionclaw::interface::Point3D> legion_stopper_points_abs;
    it_parking_stoppers.stopper_points_abs(legion_stopper_points_abs);
    for (auto it_stopper_points_abs : legion_stopper_points_abs) {
      ::ros_interface::Point3D parking_info_parking_stopper_point_3d;
      parking_info_parking_stopper_point_3d.x = it_stopper_points_abs.x();
      parking_info_parking_stopper_point_3d.y = it_stopper_points_abs.y();
      parking_info_parking_stopper_point_3d.z = it_stopper_points_abs.z();
      ros_stopper_points_abs.emplace_back(
          parking_info_parking_stopper_point_3d);
    }
    parking_info_parking_stopper.stopper_points_abs = ros_stopper_points_abs;
    ros_parking_stoppers.emplace_back(parking_info_parking_stopper);
  }
  parking_info.parking_stoppers = ros_parking_stoppers;
  parking_info.parking_direction_type = msg.parking_direction_type();
  parking_info.parking_source_type = msg.parking_source_type();

  parking_info_pub_.publish(parking_info);
}

template <typename T>
void RosMessageManager<T>::PublishStopInfo(legionclaw::interface::StopInfo msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::StopInfo stop_info;
  MESSAGE_HEADER_ASSIGN(std_msgs, stop_info)
  std::vector<ros_interface::StopPoint> ros_stop_points;
  std::vector<legionclaw::interface::StopPoint> legion_stop_points;
  msg.stop_points(legion_stop_points);
  for (auto it_stop_points : legion_stop_points) {
    ::ros_interface::StopPoint stop_info_stop_point;
    ::ros_interface::Point3D stop_info_stop_point_point;
    stop_info_stop_point_point.x = it_stop_points.point().x();
    stop_info_stop_point_point.y = it_stop_points.point().y();
    stop_info_stop_point_point.z = it_stop_points.point().z();
    stop_info_stop_point.point = stop_info_stop_point_point;
    stop_info_stop_point.theta = it_stop_points.theta();
    stop_info_stop_point.type = it_stop_points.type();
    stop_info_stop_point.stop_distance = it_stop_points.stop_distance();
    ros_stop_points.emplace_back(stop_info_stop_point);
  }
  stop_info.stop_points = ros_stop_points;

  stop_info_pub_.publish(stop_info);
}

template <typename T>
void RosMessageManager<T>::PublishGlobalRouteMsg(
    legionclaw::interface::GlobalRouteMsg msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::GlobalRouteMsg global_route_msg;
  MESSAGE_HEADER_ASSIGN(std_msgs, global_route_msg)
  std::vector<ros_interface::LaneletInfo> ros_route;
  std::vector<legionclaw::interface::LaneletInfo> legion_route;
  msg.route(legion_route);
  for (auto it_route : legion_route) {
    ::ros_interface::LaneletInfo global_route_msg_lanelet_info;
    global_route_msg_lanelet_info.lanelet_id = it_route.lanelet_id();
    global_route_msg_lanelet_info.length = it_route.length();
    ros_route.emplace_back(global_route_msg_lanelet_info);
  }
  global_route_msg.route = ros_route;
  ::ros_interface::LaneletInfo global_route_msg_current_lanelet;
  global_route_msg_current_lanelet.lanelet_id =
      msg.current_lanelet().lanelet_id();
  global_route_msg_current_lanelet.length = msg.current_lanelet().length();
  global_route_msg.current_lanelet = global_route_msg_current_lanelet;
  global_route_msg.total_mileage = msg.total_mileage();
  global_route_msg.cur_mileage = msg.cur_mileage();
  std::vector<ros_interface::LaneletInfo> ros_cur_slice;
  std::vector<legionclaw::interface::LaneletInfo> legion_cur_slice;
  msg.cur_slice(legion_cur_slice);
  for (auto it_cur_slice : legion_cur_slice) {
    ::ros_interface::LaneletInfo global_route_msg_lanelet_info;
    global_route_msg_lanelet_info.lanelet_id = it_cur_slice.lanelet_id();
    global_route_msg_lanelet_info.length = it_cur_slice.length();
    ros_cur_slice.emplace_back(global_route_msg_lanelet_info);
  }
  global_route_msg.cur_slice = ros_cur_slice;

  global_route_msg_pub_.publish(global_route_msg);
}

template <typename T>
void RosMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(std_msgs, faults)
  FAULTS_PARSER_1(ros, faults)

  faults_pub_.publish(faults);
}

template <typename T>
void RosMessageManager<T>::PublishPolygon2D(legionclaw::interface::Polygon2D msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::Polygon2D polygon_2d;
  polygon_2d.coordinate_system = msg.coordinate_system();
  std::vector<ros_interface::Point2D> ros_points;
  std::vector<legionclaw::interface::Point2D> legion_points;
  msg.points(legion_points);
  for (auto it_points : legion_points) {
    ::ros_interface::Point2D polygon_2d_point_2d;
    polygon_2d_point_2d.x = it_points.x();
    polygon_2d_point_2d.y = it_points.y();
    ros_points.emplace_back(polygon_2d_point_2d);
  }
  polygon_2d.points = ros_points;

  polygon_2d_pub_.publish(polygon_2d);
}

template <typename T>
void RosMessageManager<T>::PublishTrafficEventsOutput(
    legionclaw::interface::TrafficEvents msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::TrafficEvents traffic_events;
  MESSAGE_HEADER_ASSIGN(std_msgs, traffic_events)
  ::ros_interface::RouteFusionInfo traffic_events_route_fusion_info;
  traffic_events_route_fusion_info.fusion_flag =
      msg.route_fusion_info().fusion_flag();
  traffic_events_route_fusion_info.fusion_reason =
      msg.route_fusion_info().fusion_reason();
  traffic_events.route_fusion_info = traffic_events_route_fusion_info;
  ::ros_interface::JunctionInfo traffic_events_junction_info;
  traffic_events_junction_info.id = msg.junction_info().id();
  traffic_events_junction_info.light_flag = msg.junction_info().light_flag();
  traffic_events_junction_info.light_color = msg.junction_info().light_color();
  traffic_events_junction_info.light_remain_time =
      msg.junction_info().light_remain_time();
  traffic_events_junction_info.distance_to_stop =
      msg.junction_info().distance_to_stop();
  traffic_events_junction_info.direction_flag =
      msg.junction_info().direction_flag();
  traffic_events_junction_info.direction = msg.junction_info().direction();
  traffic_events_junction_info.distance_to_junction =
      msg.junction_info().distance_to_junction();
  std::vector<ros_interface::Point3D> ros_stop_line;
  std::vector<legionclaw::interface::Point3D> legion_stop_line;
  msg.junction_info().stop_line(legion_stop_line);
  for (auto it_stop_line : legion_stop_line) {
    ::ros_interface::Point3D traffic_events_junction_info_point_3d;
    traffic_events_junction_info_point_3d.x = it_stop_line.x();
    traffic_events_junction_info_point_3d.y = it_stop_line.y();
    traffic_events_junction_info_point_3d.z = it_stop_line.z();
    ros_stop_line.emplace_back(traffic_events_junction_info_point_3d);
  }
  traffic_events_junction_info.stop_line = ros_stop_line;
  traffic_events.junction_info = traffic_events_junction_info;
  ::ros_interface::LimitSpeedInfo traffic_events_limit_speed_info;
  traffic_events_limit_speed_info.limitspeed_valid_flag =
      msg.limit_speed_info().limitspeed_valid_flag();
  traffic_events_limit_speed_info.limit_speed =
      msg.limit_speed_info().limit_speed();
  traffic_events_limit_speed_info.limit_distance =
      msg.limit_speed_info().limit_distance();
  traffic_events.limit_speed_info = traffic_events_limit_speed_info;

  traffic_events_output_pub_.publish(traffic_events);
}

template <typename T>
void RosMessageManager<T>::HandleTrafficLightMsgMessage(
    const ros_interface::TrafficLightMsg& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::TrafficLightMsg* msg_obj_ptr = &msg_obj;
  ros_interface::TrafficLightMsg* msg =
      const_cast<ros_interface::TrafficLightMsg*>(msg_obj_ptr);

  legionclaw::interface::TrafficLightMsg traffic_light_msg;
  MESSAGE_HEADER_PARSER(traffic_light_msg)
  std::vector<legionclaw::interface::TrafficLight> traffic_light;
  for (auto it_traffic_light : msg->traffic_light) {
    legionclaw::interface::TrafficLight traffic_light_msg_traffic_light;
    traffic_light_msg_traffic_light.set_color(
        (legionclaw::common::TrafficLightColor)it_traffic_light.color);
    traffic_light_msg_traffic_light.set_id(it_traffic_light.id);
    traffic_light_msg_traffic_light.set_type(
        (legionclaw::common::TrafficLightType)it_traffic_light.type);
    traffic_light_msg_traffic_light.set_confidence(it_traffic_light.confidence);
    legionclaw::interface::ImageRect traffic_light_msg_traffic_light_light_rect;
    traffic_light_msg_traffic_light_light_rect.set_x(
        it_traffic_light.light_rect.x);
    traffic_light_msg_traffic_light_light_rect.set_y(
        it_traffic_light.light_rect.y);
    traffic_light_msg_traffic_light_light_rect.set_width(
        it_traffic_light.light_rect.width);
    traffic_light_msg_traffic_light_light_rect.set_height(
        it_traffic_light.light_rect.height);
    traffic_light_msg_traffic_light.set_light_rect(
        traffic_light_msg_traffic_light_light_rect);
    legionclaw::interface::Point3D traffic_light_msg_traffic_light_position;
    traffic_light_msg_traffic_light_position.set_x(it_traffic_light.position.x);
    traffic_light_msg_traffic_light_position.set_y(it_traffic_light.position.y);
    traffic_light_msg_traffic_light_position.set_z(it_traffic_light.position.z);
    traffic_light_msg_traffic_light.set_position(
        traffic_light_msg_traffic_light_position);
    traffic_light_msg_traffic_light.set_distance(it_traffic_light.distance);
    std::vector<int32_t> light_lanes;
    for (auto it_light_lanes : it_traffic_light.light_lanes) {
      int32_t light_lanes_item;
      light_lanes_item = it_light_lanes;
      light_lanes.emplace_back(light_lanes_item);
    }
    traffic_light_msg_traffic_light.set_light_lanes(&light_lanes);
    traffic_light_msg_traffic_light.set_tracking_time(
        it_traffic_light.tracking_time);
    traffic_light_msg_traffic_light.set_blink(it_traffic_light.blink);
    traffic_light_msg_traffic_light.set_blinking_time(
        it_traffic_light.blinking_time);
    traffic_light_msg_traffic_light.set_remaining_time(
        it_traffic_light.remaining_time);
    legionclaw::interface::Time traffic_light_msg_traffic_light_create_time;
    traffic_light_msg_traffic_light_create_time.set_sec(
        it_traffic_light.create_time.sec);
    traffic_light_msg_traffic_light_create_time.set_nsec(
        it_traffic_light.create_time.nsec);
    traffic_light_msg_traffic_light.set_create_time(
        traffic_light_msg_traffic_light_create_time);
    traffic_light.emplace_back(traffic_light_msg_traffic_light);
  }
  traffic_light_msg.set_traffic_light(&traffic_light);
  legionclaw::interface::TrafficLightDebug traffic_light_msg_traffic_light_debug;
  legionclaw::interface::TrafficLightBox
      traffic_light_msg_traffic_light_debug_cropbox;
  traffic_light_msg_traffic_light_debug_cropbox.set_x(
      msg->traffic_light_debug.cropbox.x);
  traffic_light_msg_traffic_light_debug_cropbox.set_y(
      msg->traffic_light_debug.cropbox.y);
  traffic_light_msg_traffic_light_debug_cropbox.set_width(
      msg->traffic_light_debug.cropbox.width);
  traffic_light_msg_traffic_light_debug_cropbox.set_height(
      msg->traffic_light_debug.cropbox.height);
  traffic_light_msg_traffic_light_debug_cropbox.set_color(
      (legionclaw::common::TrafficLightColor)
          msg->traffic_light_debug.cropbox.color);
  traffic_light_msg_traffic_light_debug_cropbox.set_selected(
      msg->traffic_light_debug.cropbox.selected);
  traffic_light_msg_traffic_light_debug_cropbox.set_camera_name(
      msg->traffic_light_debug.cropbox.camera_name);
  traffic_light_msg_traffic_light_debug.set_cropbox(
      traffic_light_msg_traffic_light_debug_cropbox);
  std::vector<legionclaw::interface::TrafficLightBox> box;
  for (auto it_box : msg->traffic_light_debug.box) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(it_box.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(it_box.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_box.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_box.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_box.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_box.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_box.camera_name);
    box.emplace_back(traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_box(&box);
  traffic_light_msg_traffic_light_debug.set_signal_num(
      msg->traffic_light_debug.signal_num);
  traffic_light_msg_traffic_light_debug.set_valid_pos(
      msg->traffic_light_debug.valid_pos);
  traffic_light_msg_traffic_light_debug.set_ts_diff_pos(
      msg->traffic_light_debug.ts_diff_pos);
  traffic_light_msg_traffic_light_debug.set_ts_diff_sys(
      msg->traffic_light_debug.ts_diff_sys);
  traffic_light_msg_traffic_light_debug.set_project_error(
      msg->traffic_light_debug.project_error);
  traffic_light_msg_traffic_light_debug.set_distance_to_stop_line(
      msg->traffic_light_debug.distance_to_stop_line);
  traffic_light_msg_traffic_light_debug.set_camera_id(
      msg->traffic_light_debug.camera_id);
  std::vector<legionclaw::interface::TrafficLightBox> crop_roi;
  for (auto it_crop_roi : msg->traffic_light_debug.crop_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_crop_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_crop_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_crop_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_crop_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_crop_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_crop_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_crop_roi.camera_name);
    crop_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_crop_roi(&crop_roi);
  std::vector<legionclaw::interface::TrafficLightBox> projected_roi;
  for (auto it_projected_roi : msg->traffic_light_debug.projected_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_projected_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_projected_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_projected_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_projected_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_projected_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_projected_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_projected_roi.camera_name);
    projected_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_projected_roi(&projected_roi);
  std::vector<legionclaw::interface::TrafficLightBox> rectified_roi;
  for (auto it_rectified_roi : msg->traffic_light_debug.rectified_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_rectified_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_rectified_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_rectified_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_rectified_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_rectified_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_rectified_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_rectified_roi.camera_name);
    rectified_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_rectified_roi(&rectified_roi);
  std::vector<legionclaw::interface::TrafficLightBox> debug_roi;
  for (auto it_debug_roi : msg->traffic_light_debug.debug_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_debug_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_debug_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_debug_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_debug_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_debug_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_debug_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_debug_roi.camera_name);
    debug_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_debug_roi(&debug_roi);
  traffic_light_msg.set_traffic_light_debug(
      traffic_light_msg_traffic_light_debug);
  traffic_light_msg.set_contain_lights(msg->contain_lights);
  traffic_light_msg.set_camera_id(
      (legionclaw::interface::TrafficLightMsg::CameraID)msg->camera_id);
  traffic_light_msg.set_is_valid(msg->is_valid);

  instance_->HandleTrafficLightMsg(traffic_light_msg);
}

template <typename T>
void RosMessageManager<T>::HandleLocationMessage(
    const ros_interface::Location& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::Location* msg_obj_ptr = &msg_obj;
  ros_interface::Location* msg =
      const_cast<ros_interface::Location*>(msg_obj_ptr);

  legionclaw::interface::Location location;
  MESSAGE_HEADER_PARSER(location)
  legionclaw::interface::PointLLH location_position;
  location_position.set_lon(msg->position.lon);
  location_position.set_lat(msg->position.lat);
  location_position.set_height(msg->position.height);
  location.set_position(location_position);
  location.set_pitch(msg->pitch);
  location.set_roll(msg->roll);
  location.set_heading(msg->heading);
  legionclaw::interface::Point3D location_linear_velocity;
  location_linear_velocity.set_x(msg->linear_velocity.x);
  location_linear_velocity.set_y(msg->linear_velocity.y);
  location_linear_velocity.set_z(msg->linear_velocity.z);
  location.set_linear_velocity(location_linear_velocity);
  legionclaw::interface::Point3D location_linear_acceleration;
  location_linear_acceleration.set_x(msg->linear_acceleration.x);
  location_linear_acceleration.set_y(msg->linear_acceleration.y);
  location_linear_acceleration.set_z(msg->linear_acceleration.z);
  location.set_linear_acceleration(location_linear_acceleration);
  legionclaw::interface::Point3D location_angular_velocity;
  location_angular_velocity.set_x(msg->angular_velocity.x);
  location_angular_velocity.set_y(msg->angular_velocity.y);
  location_angular_velocity.set_z(msg->angular_velocity.z);
  location.set_angular_velocity(location_angular_velocity);
  location.set_rtk_flag((legionclaw::interface::Location::RTKFlag)msg->rtk_flag);
  location.set_odom_type((legionclaw::interface::Location::OdomType)msg->odom_type);
  location.set_auxiliary_type(
      (legionclaw::interface::Location::AuxiliaryType)msg->auxiliary_type);
  location.set_location_valid_flag(
      (legionclaw::common::IsValid)msg->location_valid_flag);
  location.set_origin_lat(msg->origin_lat);
  location.set_origin_lon(msg->origin_lon);
  legionclaw::interface::PointENU location_utm_position;
  location_utm_position.set_x(msg->utm_position.x);
  location_utm_position.set_y(msg->utm_position.y);
  location_utm_position.set_z(msg->utm_position.z);
  location.set_utm_position(location_utm_position);
  location.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  legionclaw::interface::PointENU location_utm_position_next;
  location_utm_position_next.set_x(msg->utm_position_next.x);
  location_utm_position_next.set_y(msg->utm_position_next.y);
  location_utm_position_next.set_z(msg->utm_position_next.z);
  location.set_utm_position_next(location_utm_position_next);
  legionclaw::interface::Point3D location_position_std_dev;
  location_position_std_dev.set_x(msg->position_std_dev.x);
  location_position_std_dev.set_y(msg->position_std_dev.y);
  location_position_std_dev.set_z(msg->position_std_dev.z);
  location.set_position_std_dev(location_position_std_dev);
  legionclaw::interface::Point3D location_orientation_std_dev;
  location_orientation_std_dev.set_x(msg->orientation_std_dev.x);
  location_orientation_std_dev.set_y(msg->orientation_std_dev.y);
  location_orientation_std_dev.set_z(msg->orientation_std_dev.z);
  location.set_orientation_std_dev(location_orientation_std_dev);
  legionclaw::interface::Point3D location_linear_velocity_std_dev;
  location_linear_velocity_std_dev.set_x(msg->linear_velocity_std_dev.x);
  location_linear_velocity_std_dev.set_y(msg->linear_velocity_std_dev.y);
  location_linear_velocity_std_dev.set_z(msg->linear_velocity_std_dev.z);
  location.set_linear_velocity_std_dev(location_linear_velocity_std_dev);
  legionclaw::interface::Point3D location_linear_acceleration_std_dev;
  location_linear_acceleration_std_dev.set_x(
      msg->linear_acceleration_std_dev.x);
  location_linear_acceleration_std_dev.set_y(
      msg->linear_acceleration_std_dev.y);
  location_linear_acceleration_std_dev.set_z(
      msg->linear_acceleration_std_dev.z);
  location.set_linear_acceleration_std_dev(
      location_linear_acceleration_std_dev);
  legionclaw::interface::Point3D location_angular_velocity_std_dev;
  location_angular_velocity_std_dev.set_x(msg->angular_velocity_std_dev.x);
  location_angular_velocity_std_dev.set_y(msg->angular_velocity_std_dev.y);
  location_angular_velocity_std_dev.set_z(msg->angular_velocity_std_dev.z);
  location.set_angular_velocity_std_dev(location_angular_velocity_std_dev);
  instance_->HandleLocation(location);
}

template <typename T>
void RosMessageManager<T>::HandleChassisMessage(
    const ros_interface::Chassis& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::Chassis* msg_obj_ptr = &msg_obj;
  ros_interface::Chassis* msg =
      const_cast<ros_interface::Chassis*>(msg_obj_ptr);

  legionclaw::interface::Chassis chassis;
  MESSAGE_HEADER_PARSER(chassis)
  chassis.set_moving_status((legionclaw::common::MovingStatus)msg->moving_status);
  chassis.set_driving_mode((legionclaw::common::DrivingMode)msg->driving_mode);
  chassis.set_steer_driving_mode(
      (legionclaw::common::DrivingMode)msg->steer_driving_mode);
  chassis.set_steering_status(
      (legionclaw::common::ControlStatus)msg->steering_status);
  chassis.set_front_steering_value(msg->front_steering_value);
  chassis.set_rear_steering_value(msg->rear_steering_value);
  chassis.set_steering_torque_nm(msg->steering_torque_nm);
  chassis.set_front_steering_rate_dps(msg->front_steering_rate_dps);
  chassis.set_rear_steering_rate_dps(msg->rear_steering_rate_dps);
  chassis.set_accel_driving_mode(
      (legionclaw::common::DrivingMode)msg->accel_driving_mode);
  chassis.set_accel_status((legionclaw::common::ControlStatus)msg->accel_status);
  chassis.set_accel_value(msg->accel_value);
  chassis.set_brake_driving_mode(
      (legionclaw::common::DrivingMode)msg->brake_driving_mode);
  chassis.set_brake_status((legionclaw::common::ControlStatus)msg->brake_status);
  chassis.set_brake_value(msg->brake_value);
  chassis.set_backup_brake_driving_mode(
      (legionclaw::common::DrivingMode)msg->backup_brake_driving_mode);
  chassis.set_backup_brake_status(
      (legionclaw::common::ControlStatus)msg->backup_brake_status);
  chassis.set_backup_brake_value(msg->backup_brake_value);
  chassis.set_epb_driving_mode(
      (legionclaw::common::DrivingMode)msg->epb_driving_mode);
  chassis.set_epb_status((legionclaw::common::ControlStatus)msg->epb_status);
  chassis.set_epb_level((legionclaw::common::EPBLevel)msg->epb_level);
  chassis.set_engine_status((legionclaw::common::EngineStauts)msg->engine_status);
  chassis.set_engine_rpm(msg->engine_rpm);
  chassis.set_engine_torque(msg->engine_torque);
  chassis.set_speed_mps(msg->speed_mps);
  chassis.set_odometer_m(msg->odometer_m);
  chassis.set_fuel_range_m(msg->fuel_range_m);
  chassis.set_gear_driving_mode(
      (legionclaw::common::DrivingMode)msg->gear_driving_mode);
  chassis.set_gear_status((legionclaw::common::ControlStatus)msg->gear_status);
  chassis.set_gear_location((legionclaw::common::GearPosition)msg->gear_location);
  chassis.set_driver_seat_belt(
      (legionclaw::common::SwitchStatus)msg->driver_seat_belt);
  chassis.set_high_beam_status(
      (legionclaw::common::SwitchStatus)msg->high_beam_status);
  chassis.set_low_beam_status(
      (legionclaw::common::SwitchStatus)msg->low_beam_status);
  chassis.set_horn_status((legionclaw::common::SwitchStatus)msg->horn_status);
  chassis.set_turn_lamp_status(
      (legionclaw::common::TurnSignal)msg->turn_lamp_status);
  chassis.set_front_wiper_status(
      (legionclaw::common::SwitchStatus)msg->front_wiper_status);
  chassis.set_rear_wiper_status(
      (legionclaw::common::SwitchStatus)msg->rear_wiper_status);
  chassis.set_position_lamp_status(
      (legionclaw::common::SwitchStatus)msg->position_lamp_status);
  chassis.set_front_fog_lamp_status(
      (legionclaw::common::SwitchStatus)msg->front_fog_lamp_status);
  chassis.set_rear_fog_lamp_status(
      (legionclaw::common::SwitchStatus)msg->rear_fog_lamp_status);
  chassis.set_brake_lamp_status(
      (legionclaw::common::SwitchStatus)msg->brake_lamp_status);
  chassis.set_alarm_lamp_status(
      (legionclaw::common::SwitchStatus)msg->alarm_lamp_status);
  chassis.set_lf_door_status((legionclaw::common::DoorStatus)msg->lf_door_status);
  chassis.set_rf_door_status((legionclaw::common::DoorStatus)msg->rf_door_status);
  chassis.set_lr_door_status((legionclaw::common::DoorStatus)msg->lr_door_status);
  chassis.set_rr_door_status((legionclaw::common::DoorStatus)msg->rr_door_status);
  chassis.set_rearview_mirror_status(
      (legionclaw::common::FoldUnfoldStatus)msg->rearview_mirror_status);
  chassis.set_trunk_status((legionclaw::common::DoorStatus)msg->trunk_status);
  chassis.set_engine_bay_door_status(
      (legionclaw::common::DoorStatus)msg->engine_bay_door_status);
  chassis.set_wheel_direction_rr(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_rr);
  chassis.set_wheel_spd_rr(msg->wheel_spd_rr);
  chassis.set_wheel_direction_rl(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_rl);
  chassis.set_wheel_spd_rl(msg->wheel_spd_rl);
  chassis.set_wheel_direction_fr(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_fr);
  chassis.set_wheel_spd_fr(msg->wheel_spd_fr);
  chassis.set_wheel_direction_fl(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_fl);
  chassis.set_wheel_spd_fl(msg->wheel_spd_fl);
  chassis.set_is_tire_pressure_ok(
      (legionclaw::common::FailureStatus)msg->is_tire_pressure_ok);
  chassis.set_is_tire_pressure_lf_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_lf_valid);
  chassis.set_tire_pressure_lf(msg->tire_pressure_lf);
  chassis.set_is_tire_pressure_rf_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_rf_valid);
  chassis.set_tire_pressure_rf(msg->tire_pressure_rf);
  chassis.set_is_tire_pressure_lr_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_lr_valid);
  chassis.set_tire_pressure_lr(msg->tire_pressure_lr);
  chassis.set_is_tire_pressure_rr_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_rr_valid);
  chassis.set_tire_pressure_rr(msg->tire_pressure_rr);
  chassis.set_battery_power_percentage(msg->battery_power_percentage);
  chassis.set_air_bag_status(
      (legionclaw::common::FailureStatus)msg->air_bag_status);
  chassis.set_charging_gun_status(
      (legionclaw::common::PlugStatus)msg->charging_gun_status);
  chassis.set_vehicle_power_status(
      (legionclaw::common::FailureStatus)msg->vehicle_power_status);
  std::vector<legionclaw::interface::Chassis::ErrorCode> chassis_error_code;
  for (auto it_chassis_error_code : msg->chassis_error_code) {
    legionclaw::interface::Chassis::ErrorCode error_code;
    error_code = (legionclaw::interface::Chassis::ErrorCode)it_chassis_error_code;
    chassis_error_code.emplace_back(error_code);
  }
  chassis.set_chassis_error_code(&chassis_error_code);

  instance_->HandleChassis(chassis);
}

template <typename T>
void RosMessageManager<T>::HandleObuCmdMsgMessage(
    const ros_interface::ObuCmdMsg& msg_obj) {
  if (is_init_ == false)
    return;
  const ros_interface::ObuCmdMsg* msg_obj_ptr = &msg_obj;
  ros_interface::ObuCmdMsg* msg =
      const_cast<ros_interface::ObuCmdMsg*>(msg_obj_ptr);

  legionclaw::interface::ObuCmdMsg obu_cmd_msg;
  MESSAGE_HEADER_PARSER(obu_cmd_msg)
  obu_cmd_msg.set_id(msg->id);
  obu_cmd_msg.set_name(msg->name);
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  for (auto it_obu_cmd_list : msg->obu_cmd_list) {
    legionclaw::interface::ObuCmd obu_cmd_msg_obu_cmd;
    obu_cmd_msg_obu_cmd.set_code(it_obu_cmd_list.code);
    obu_cmd_msg_obu_cmd.set_val(it_obu_cmd_list.val);
    obu_cmd_list.emplace_back(obu_cmd_msg_obu_cmd);
  }
  obu_cmd_msg.set_obu_cmd_list(&obu_cmd_list);

  instance_->HandleObuCmdMsg(obu_cmd_msg);
}

template <typename T>
void RosMessageManager<T>::HandleTrafficEventsInputMessage(
    const ros_interface::TrafficEvents& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::TrafficEvents* msg_obj_ptr = &msg_obj;
  ros_interface::TrafficEvents* msg =
      const_cast<ros_interface::TrafficEvents*>(msg_obj_ptr);

  legionclaw::interface::TrafficEvents traffic_events;
  MESSAGE_HEADER_PARSER(traffic_events)
  legionclaw::interface::RouteFusionInfo traffic_events_route_fusion_info;
  traffic_events_route_fusion_info.set_fusion_flag(
      (legionclaw::common::IsValid)msg->route_fusion_info.fusion_flag);
  traffic_events_route_fusion_info.set_fusion_reason(
      msg->route_fusion_info.fusion_reason);
  traffic_events.set_route_fusion_info(traffic_events_route_fusion_info);
  legionclaw::interface::JunctionInfo traffic_events_junction_info;
  traffic_events_junction_info.set_id(msg->junction_info.id);
  traffic_events_junction_info.set_light_flag(
      (legionclaw::common::IsValid)msg->junction_info.light_flag);
  traffic_events_junction_info.set_light_color(
      (legionclaw::common::TrafficLightColor)msg->junction_info.light_color);
  traffic_events_junction_info.set_light_remain_time(
      msg->junction_info.light_remain_time);
  traffic_events_junction_info.set_distance_to_stop(
      msg->junction_info.distance_to_stop);
  traffic_events_junction_info.set_direction_flag(
      (legionclaw::common::IsValid)msg->junction_info.direction_flag);
  traffic_events_junction_info.set_direction(
      (legionclaw::common::Direction)msg->junction_info.direction);
  traffic_events_junction_info.set_distance_to_junction(
      msg->junction_info.distance_to_junction);
  std::vector<legionclaw::interface::Point3D> stop_line;
  for (auto it_stop_line : msg->junction_info.stop_line) {
    legionclaw::interface::Point3D traffic_events_junction_info_point_3d;
    traffic_events_junction_info_point_3d.set_x(it_stop_line.x);
    traffic_events_junction_info_point_3d.set_y(it_stop_line.y);
    traffic_events_junction_info_point_3d.set_z(it_stop_line.z);
    stop_line.emplace_back(traffic_events_junction_info_point_3d);
  }
  traffic_events_junction_info.set_stop_line(&stop_line);
  traffic_events.set_junction_info(traffic_events_junction_info);
  legionclaw::interface::LimitSpeedInfo traffic_events_limit_speed_info;
  traffic_events_limit_speed_info.set_limitspeed_valid_flag(
      (legionclaw::common::IsValid)msg->limit_speed_info.limitspeed_valid_flag);
  traffic_events_limit_speed_info.set_limit_speed(
      msg->limit_speed_info.limit_speed);
  traffic_events_limit_speed_info.set_limit_distance(
      msg->limit_speed_info.limit_distance);
  traffic_events.set_limit_speed_info(traffic_events_limit_speed_info);
  instance_->HandleTrafficEventsInput(traffic_events);
}

template <typename T>
void RosMessageManager<T>::HandleOdometryMessage(
    const ros_interface::Odometry& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::Odometry* msg_obj_ptr = &msg_obj;
  ros_interface::Odometry* msg =
      const_cast<ros_interface::Odometry*>(msg_obj_ptr);

  legionclaw::interface::Odometry odometry;
  MESSAGE_HEADER_PARSER(odometry)
  legionclaw::interface::PointENU odometry_position;
  odometry_position.set_x(msg->position.x);
  odometry_position.set_y(msg->position.y);
  odometry_position.set_z(msg->position.z);
  odometry.set_position(odometry_position);
  legionclaw::interface::Quaternion odometry_orientation;
  odometry_orientation.set_qx(msg->orientation.qx);
  odometry_orientation.set_qy(msg->orientation.qy);
  odometry_orientation.set_qz(msg->orientation.qz);
  odometry_orientation.set_qw(msg->orientation.qw);
  odometry.set_orientation(odometry_orientation);
  std::vector<double> covariance;
  for (auto it_covariance : msg->covariance) {
    double covariance_item;
    covariance_item = it_covariance;
    covariance.emplace_back(covariance_item);
  }
  odometry.set_covariance(&covariance);

  instance_->HandleOdometry(odometry);
}

template <typename T>
void RosMessageManager<T>::HandleRoutingRequestMessage(
    const ros_interface::RoutingRequest& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::RoutingRequest* msg_obj_ptr = &msg_obj;
  ros_interface::RoutingRequest* msg =
      const_cast<ros_interface::RoutingRequest*>(msg_obj_ptr);

  legionclaw::interface::RoutingRequest routing_request;
  MESSAGE_HEADER_PARSER(routing_request)
  routing_request.set_request_source(msg->request_source);
  routing_request.set_request_type(msg->request_type);
  routing_request.set_num_of_kp(msg->num_of_kp);
  std::vector<legionclaw::interface::KeyPoint> key_point_list;
  for (auto it_key_point_list : msg->key_point_list) {
    legionclaw::interface::KeyPoint routing_request_key_point;
    routing_request_key_point.set_id(it_key_point_list.id);
    routing_request_key_point.set_latitude(it_key_point_list.latitude);
    routing_request_key_point.set_longitude(it_key_point_list.longitude);
    routing_request_key_point.set_ele(it_key_point_list.ele);
    routing_request_key_point.set_heading(it_key_point_list.heading);
    routing_request_key_point.set_name(it_key_point_list.name);
    key_point_list.emplace_back(routing_request_key_point);
  }
  routing_request.set_key_point_list(&key_point_list);

  instance_->HandleRoutingRequest(routing_request);
}

template <typename T> void RosMessageManager<T>::RegisterLoop() {
  //   eprosima::fastrtps::Duration_t timeout(0.1, 0);
  while (true) {
    std::unique_lock<std::mutex> lock(mode_mutex_);
    switch (action_mode_) {
    case MessageActionMode::DO_NOTHING:
      /* code sleep */
      break;
    case MessageActionMode::TO_ACTIVATE:
      TaskStart();

      break;
    case MessageActionMode::TO_DEACTIVATE:
      TaskStop();
      break;
    default:
      // sleep
      break;
    }
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <typename T> void RosMessageManager<T>::Run() {
  ros::spin();
  ros::shutdown();
  exit(0);
}

template <typename T> void RosMessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace routing
} // namespace legionclaw
#endif
