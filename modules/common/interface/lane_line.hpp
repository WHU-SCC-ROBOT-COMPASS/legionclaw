/**
 * @file    lane_line.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <vector>

#include "modules/common/interface/end_points.hpp"
#include "modules/common/interface/lane_line_cubic_curve.hpp"
#include "modules/common/interface/point_2d.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/time.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LaneLine {
public:
  LaneLine() {
    pts_vehicle_mutex_ = std::make_shared<std::mutex>();
    pts_image_mutex_ = std::make_shared<std::mutex>();
    pts_abs_mutex_ = std::make_shared<std::mutex>();
    pts_key_mutex_ = std::make_shared<std::mutex>();
    homography_mat_mutex_ = std::make_shared<std::mutex>();
    homography_mat_inv_mutex_ = std::make_shared<std::mutex>();

    lane_type_ = legionclaw::interface::LaneLine::LaneType::SOLID;
    lane_color_ = legionclaw::interface::LaneLine::LaneColor::WHITE;
    pos_type_ = legionclaw::interface::LaneLine::PosType::BOLLARD_LEFT;
    clear_pts_vehicle();
    clear_pts_image();
    clear_pts_abs();
    clear_pts_key();
    hd_lane_id_ = 0;
    confidence_ = 0.0;
    lane_quality_ = legionclaw::interface::LaneLine::LaneQuality::VERY_LOW;
    fused_lane_type_ =
        legionclaw::interface::LaneLine::FusedLaneType::FUSED_LANE_TYPE_CAMERA;
    clear_homography_mat();
    clear_homography_mat_inv();
    lane_coordinate_type_ =
        legionclaw::interface::LaneLine::LaneCoordinateType::FRAME;
    use_type_ = legionclaw::interface::LaneLine::UseType::REAL;
  }
  ~LaneLine() = default;

  enum LaneType {
    SOLID = 0,
    DASHED = 1,
    ROAD_DELIMITER = 2,
    NONE = 3,
    LANE_TYPE_UNKNOWN = 4,
    CURB = 5,
  };

  enum LaneColor {
    WHITE = 0,
    YELLOW = 1,
    OTHERS = 2,
    LANE_COLOR_UNKNOWN = 3,
  };

  enum PosType {
    BOLLARD_LEFT = -5,
    FOURTH_LEFT = -4,
    THIRD_LEFT = -3,
    ADJACENT_LEFT = -2,
    EGO_LEFT = -1,
    EGO_RIGHT = 1,
    ADJACENT_RIGHT = 2,
    THIRD_RIGHT = 3,
    FOURTH_RIGHT = 4,
    BOLLARD_RIGHT = 5,
    OTHER = 6,
    POS_TYPE_UNKNOWN = 7,
  };

  enum LaneQuality {
    VERY_LOW = 0,
    LOW = 1,
    HIGH = 2,
    VERY_HIGH = 3,
  };

  enum FusedLaneType {
    FUSED_LANE_TYPE_CAMERA = 0,
    HD = 1,
    FUSED = 2,
    FUSED_LANE_TYPE_UNKNOWN = 3,
  };

  enum LaneCoordinateType {
    FRAME = 0,
    IMAGE = 1,
    LANE_COORDINATE_TYPE_CAMERA = 2,
    VEHICLE = 3,
    ABSOLUTE = 4,
  };

  enum UseType {
    REAL = 0,
    VIRTUAL = 1,
  };

  inline void
  set_lane_type(const legionclaw::interface::LaneLine::LaneType &lane_type) {
    lane_type_ = lane_type;
    lane_type_ptr_ = &lane_type_;
  }

  inline const legionclaw::interface::LaneLine::LaneType &lane_type() const {
    return lane_type_;
  }

  inline legionclaw::interface::LaneLine::LaneType *mutable_lane_type() {
    return &lane_type_;
  }

  inline bool has_lane_type() { return (lane_type_ptr_ != nullptr); }

  inline void
  set_lane_color(const legionclaw::interface::LaneLine::LaneColor &lane_color) {
    lane_color_ = lane_color;
    lane_color_ptr_ = &lane_color_;
  }

  inline const legionclaw::interface::LaneLine::LaneColor &lane_color() const {
    return lane_color_;
  }

  inline legionclaw::interface::LaneLine::LaneColor *mutable_lane_color() {
    return &lane_color_;
  }

  inline bool has_lane_color() { return (lane_color_ptr_ != nullptr); }

  inline void
  set_pos_type(const legionclaw::interface::LaneLine::PosType &pos_type) {
    pos_type_ = pos_type;
    pos_type_ptr_ = &pos_type_;
  }

  inline const legionclaw::interface::LaneLine::PosType &pos_type() const {
    return pos_type_;
  }

  inline legionclaw::interface::LaneLine::PosType *mutable_pos_type() {
    return &pos_type_;
  }

  inline bool has_pos_type() { return (pos_type_ptr_ != nullptr); }

  inline void
  set_curve_vehicle(const legionclaw::interface::LaneLineCubicCurve &curve_vehicle) {
    curve_vehicle_ = curve_vehicle;
    curve_vehicle_ptr_ = &curve_vehicle_;
  }

  inline const legionclaw::interface::LaneLineCubicCurve &curve_vehicle() const {
    return curve_vehicle_;
  }

  inline legionclaw::interface::LaneLineCubicCurve *mutable_curve_vehicle() {
    return &curve_vehicle_;
  }

  inline bool has_curve_vehicle() { return (curve_vehicle_ptr_ != nullptr); }

  inline void
  set_curve_image(const legionclaw::interface::LaneLineCubicCurve &curve_image) {
    curve_image_ = curve_image;
    curve_image_ptr_ = &curve_image_;
  }

  inline const legionclaw::interface::LaneLineCubicCurve &curve_image() const {
    return curve_image_;
  }

  inline legionclaw::interface::LaneLineCubicCurve *mutable_curve_image() {
    return &curve_image_;
  }

  inline bool has_curve_image() { return (curve_image_ptr_ != nullptr); }

  inline void
  set_curve_abs(const legionclaw::interface::LaneLineCubicCurve &curve_abs) {
    curve_abs_ = curve_abs;
    curve_abs_ptr_ = &curve_abs_;
  }

  inline const legionclaw::interface::LaneLineCubicCurve &curve_abs() const {
    return curve_abs_;
  }

  inline legionclaw::interface::LaneLineCubicCurve *mutable_curve_abs() {
    return &curve_abs_;
  }

  inline bool has_curve_abs() { return (curve_abs_ptr_ != nullptr); }

  inline void
  set_pts_vehicle(std::vector<legionclaw::interface::Point3D> *pts_vehicle) {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle_.assign(pts_vehicle->begin(), pts_vehicle->end());
  }

  inline void
  set_pts_vehicle(const std::vector<legionclaw::interface::Point3D> &pts_vehicle) {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle_ = pts_vehicle;
  }

  inline void set_pts_vehicle(const uint32_t index,
                              legionclaw::interface::Point3D &pts_vehicle) {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle_[index] = pts_vehicle;
  }

  inline void add_pts_vehicle(const legionclaw::interface::Point3D &pts_vehicle) {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle_.emplace_back(pts_vehicle);
  }

  inline const legionclaw::interface::Point3D &pts_vehicle(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    return pts_vehicle_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_pts_vehicle() {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    return &pts_vehicle_;
  }

  inline void
  pts_vehicle(std::vector<legionclaw::interface::Point3D> &pts_vehicle) const {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle.assign(pts_vehicle_.begin(), pts_vehicle_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &pts_vehicle() const {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    return pts_vehicle_;
  }

  inline uint32_t pts_vehicle_size() const {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    return pts_vehicle_.size();
  }

  inline void clear_pts_vehicle() {
    std::lock_guard<std::mutex> lock(*pts_vehicle_mutex_);
    pts_vehicle_.clear();
    pts_vehicle_.shrink_to_fit();
  }

  inline bool has_pts_vehicle() { return (pts_vehicle_size() != 0); }

  inline void set_pts_image(std::vector<legionclaw::interface::Point2D> *pts_image) {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image_.assign(pts_image->begin(), pts_image->end());
  }

  inline void
  set_pts_image(const std::vector<legionclaw::interface::Point2D> &pts_image) {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image_ = pts_image;
  }

  inline void set_pts_image(const uint32_t index,
                            legionclaw::interface::Point2D &pts_image) {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image_[index] = pts_image;
  }

  inline void add_pts_image(const legionclaw::interface::Point2D &pts_image) {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image_.emplace_back(pts_image);
  }

  inline const legionclaw::interface::Point2D &pts_image(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    return pts_image_[index];
  }

  inline std::vector<legionclaw::interface::Point2D> *mutable_pts_image() {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    return &pts_image_;
  }

  inline void
  pts_image(std::vector<legionclaw::interface::Point2D> &pts_image) const {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image.assign(pts_image_.begin(), pts_image_.end());
  }

  inline const std::vector<legionclaw::interface::Point2D> &pts_image() const {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    return pts_image_;
  }

  inline uint32_t pts_image_size() const {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    return pts_image_.size();
  }

  inline void clear_pts_image() {
    std::lock_guard<std::mutex> lock(*pts_image_mutex_);
    pts_image_.clear();
    pts_image_.shrink_to_fit();
  }

  inline bool has_pts_image() { return (pts_image_size() != 0); }

  inline void set_pts_abs(std::vector<legionclaw::interface::Point3D> *pts_abs) {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs_.assign(pts_abs->begin(), pts_abs->end());
  }

  inline void
  set_pts_abs(const std::vector<legionclaw::interface::Point3D> &pts_abs) {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs_ = pts_abs;
  }

  inline void set_pts_abs(const uint32_t index,
                          legionclaw::interface::Point3D &pts_abs) {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs_[index] = pts_abs;
  }

  inline void add_pts_abs(const legionclaw::interface::Point3D &pts_abs) {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs_.emplace_back(pts_abs);
  }

  inline const legionclaw::interface::Point3D &pts_abs(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    return pts_abs_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_pts_abs() {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    return &pts_abs_;
  }

  inline void pts_abs(std::vector<legionclaw::interface::Point3D> &pts_abs) const {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs.assign(pts_abs_.begin(), pts_abs_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &pts_abs() const {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    return pts_abs_;
  }

  inline uint32_t pts_abs_size() const {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    return pts_abs_.size();
  }

  inline void clear_pts_abs() {
    std::lock_guard<std::mutex> lock(*pts_abs_mutex_);
    pts_abs_.clear();
    pts_abs_.shrink_to_fit();
  }

  inline bool has_pts_abs() { return (pts_abs_size() != 0); }

  inline void
  set_image_end_point(const legionclaw::interface::EndPoints &image_end_point) {
    image_end_point_ = image_end_point;
    image_end_point_ptr_ = &image_end_point_;
  }

  inline const legionclaw::interface::EndPoints &image_end_point() const {
    return image_end_point_;
  }

  inline legionclaw::interface::EndPoints *mutable_image_end_point() {
    return &image_end_point_;
  }

  inline bool has_image_end_point() {
    return (image_end_point_ptr_ != nullptr);
  }

  inline void set_pts_key(std::vector<legionclaw::interface::Point2D> *pts_key) {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key_.assign(pts_key->begin(), pts_key->end());
  }

  inline void
  set_pts_key(const std::vector<legionclaw::interface::Point2D> &pts_key) {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key_ = pts_key;
  }

  inline void set_pts_key(const uint32_t index,
                          legionclaw::interface::Point2D &pts_key) {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key_[index] = pts_key;
  }

  inline void add_pts_key(const legionclaw::interface::Point2D &pts_key) {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key_.emplace_back(pts_key);
  }

  inline const legionclaw::interface::Point2D &pts_key(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    return pts_key_[index];
  }

  inline std::vector<legionclaw::interface::Point2D> *mutable_pts_key() {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    return &pts_key_;
  }

  inline void pts_key(std::vector<legionclaw::interface::Point2D> &pts_key) const {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key.assign(pts_key_.begin(), pts_key_.end());
  }

  inline const std::vector<legionclaw::interface::Point2D> &pts_key() const {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    return pts_key_;
  }

  inline uint32_t pts_key_size() const {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    return pts_key_.size();
  }

  inline void clear_pts_key() {
    std::lock_guard<std::mutex> lock(*pts_key_mutex_);
    pts_key_.clear();
    pts_key_.shrink_to_fit();
  }

  inline bool has_pts_key() { return (pts_key_size() != 0); }

  inline void set_hd_lane_id(const uint8_t &hd_lane_id) {
    hd_lane_id_ = hd_lane_id;
    hd_lane_id_ptr_ = &hd_lane_id_;
  }

  inline const uint8_t &hd_lane_id() const { return hd_lane_id_; }

  inline uint8_t *mutable_hd_lane_id() { return &hd_lane_id_; }

  inline bool has_hd_lane_id() { return (hd_lane_id_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_lane_quality(
      const legionclaw::interface::LaneLine::LaneQuality &lane_quality) {
    lane_quality_ = lane_quality;
    lane_quality_ptr_ = &lane_quality_;
  }

  inline const legionclaw::interface::LaneLine::LaneQuality &lane_quality() const {
    return lane_quality_;
  }

  inline legionclaw::interface::LaneLine::LaneQuality *mutable_lane_quality() {
    return &lane_quality_;
  }

  inline bool has_lane_quality() { return (lane_quality_ptr_ != nullptr); }

  inline void set_fused_lane_type(
      const legionclaw::interface::LaneLine::FusedLaneType &fused_lane_type) {
    fused_lane_type_ = fused_lane_type;
    fused_lane_type_ptr_ = &fused_lane_type_;
  }

  inline const legionclaw::interface::LaneLine::FusedLaneType &
  fused_lane_type() const {
    return fused_lane_type_;
  }

  inline legionclaw::interface::LaneLine::FusedLaneType *mutable_fused_lane_type() {
    return &fused_lane_type_;
  }

  inline bool has_fused_lane_type() {
    return (fused_lane_type_ptr_ != nullptr);
  }

  inline void set_homography_mat(std::vector<double> *homography_mat) {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat_.assign(homography_mat->begin(), homography_mat->end());
  }

  inline void set_homography_mat(const std::vector<double> &homography_mat) {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat_ = homography_mat;
  }

  inline void set_homography_mat(const uint32_t index, double &homography_mat) {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat_[index] = homography_mat;
  }

  inline void add_homography_mat(const double &homography_mat) {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat_.emplace_back(homography_mat);
  }

  inline const double &homography_mat(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    return homography_mat_[index];
  }

  inline std::vector<double> *mutable_homography_mat() {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    return &homography_mat_;
  }

  inline void homography_mat(std::vector<double> &homography_mat) const {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat.assign(homography_mat_.begin(), homography_mat_.end());
  }

  inline const std::vector<double> &homography_mat() const {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    return homography_mat_;
  }

  inline uint32_t homography_mat_size() const {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    return homography_mat_.size();
  }

  inline void clear_homography_mat() {
    std::lock_guard<std::mutex> lock(*homography_mat_mutex_);
    homography_mat_.clear();
    homography_mat_.shrink_to_fit();
  }

  inline bool has_homography_mat() { return (homography_mat_size() != 0); }

  inline void set_homography_mat_inv(std::vector<double> *homography_mat_inv) {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv_.assign(homography_mat_inv->begin(),
                               homography_mat_inv->end());
  }

  inline void
  set_homography_mat_inv(const std::vector<double> &homography_mat_inv) {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv_ = homography_mat_inv;
  }

  inline void set_homography_mat_inv(const uint32_t index,
                                     double &homography_mat_inv) {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv_[index] = homography_mat_inv;
  }

  inline void add_homography_mat_inv(const double &homography_mat_inv) {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv_.emplace_back(homography_mat_inv);
  }

  inline const double &homography_mat_inv(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    return homography_mat_inv_[index];
  }

  inline std::vector<double> *mutable_homography_mat_inv() {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    return &homography_mat_inv_;
  }

  inline void
  homography_mat_inv(std::vector<double> &homography_mat_inv) const {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv.assign(homography_mat_inv_.begin(),
                              homography_mat_inv_.end());
  }

  inline const std::vector<double> &homography_mat_inv() const {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    return homography_mat_inv_;
  }

  inline uint32_t homography_mat_inv_size() const {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    return homography_mat_inv_.size();
  }

  inline void clear_homography_mat_inv() {
    std::lock_guard<std::mutex> lock(*homography_mat_inv_mutex_);
    homography_mat_inv_.clear();
    homography_mat_inv_.shrink_to_fit();
  }

  inline bool has_homography_mat_inv() {
    return (homography_mat_inv_size() != 0);
  }

  inline void
  set_lane_coordinate_type(const legionclaw::interface::LaneLine::LaneCoordinateType
                               &lane_coordinate_type) {
    lane_coordinate_type_ = lane_coordinate_type;
    lane_coordinate_type_ptr_ = &lane_coordinate_type_;
  }

  inline const legionclaw::interface::LaneLine::LaneCoordinateType &
  lane_coordinate_type() const {
    return lane_coordinate_type_;
  }

  inline legionclaw::interface::LaneLine::LaneCoordinateType *
  mutable_lane_coordinate_type() {
    return &lane_coordinate_type_;
  }

  inline bool has_lane_coordinate_type() {
    return (lane_coordinate_type_ptr_ != nullptr);
  }

  inline void
  set_use_type(const legionclaw::interface::LaneLine::UseType &use_type) {
    use_type_ = use_type;
    use_type_ptr_ = &use_type_;
  }

  inline const legionclaw::interface::LaneLine::UseType &use_type() const {
    return use_type_;
  }

  inline legionclaw::interface::LaneLine::UseType *mutable_use_type() {
    return &use_type_;
  }

  inline bool has_use_type() { return (use_type_ptr_ != nullptr); }

  inline void set_create_time(const legionclaw::interface::Time &create_time) {
    create_time_ = create_time;
    create_time_ptr_ = &create_time_;
  }

  inline const legionclaw::interface::Time &create_time() const {
    return create_time_;
  }

  inline legionclaw::interface::Time *mutable_create_time() { return &create_time_; }

  inline bool has_create_time() { return (create_time_ptr_ != nullptr); }

  void operator=(const LaneLine &lane_line) { CopyFrom(lane_line); }

  void CopyFrom(const LaneLine &lane_line) {
    lane_type_ = lane_line.lane_type();
    lane_color_ = lane_line.lane_color();
    pos_type_ = lane_line.pos_type();
    curve_vehicle_ = lane_line.curve_vehicle();
    curve_image_ = lane_line.curve_image();
    curve_abs_ = lane_line.curve_abs();
    pts_vehicle_ = lane_line.pts_vehicle();
    pts_image_ = lane_line.pts_image();
    pts_abs_ = lane_line.pts_abs();
    image_end_point_ = lane_line.image_end_point();
    pts_key_ = lane_line.pts_key();
    hd_lane_id_ = lane_line.hd_lane_id();
    confidence_ = lane_line.confidence();
    lane_quality_ = lane_line.lane_quality();
    fused_lane_type_ = lane_line.fused_lane_type();
    homography_mat_ = lane_line.homography_mat();
    homography_mat_inv_ = lane_line.homography_mat_inv();
    lane_coordinate_type_ = lane_line.lane_coordinate_type();
    use_type_ = lane_line.use_type();
    create_time_ = lane_line.create_time();
  }

protected:
  std::shared_ptr<std::mutex> pts_vehicle_mutex_;
  std::shared_ptr<std::mutex> pts_image_mutex_;
  std::shared_ptr<std::mutex> pts_abs_mutex_;
  std::shared_ptr<std::mutex> pts_key_mutex_;
  std::shared_ptr<std::mutex> homography_mat_mutex_;
  std::shared_ptr<std::mutex> homography_mat_inv_mutex_;
  // 车道线类型：   SOLID = 0;   DASHED = 1;   ROAD_DELIMITER = 2;   NONE = 3;
  // UNKNOWN = 4;
  legionclaw::interface::LaneLine::LaneType lane_type_;
  legionclaw::interface::LaneLine::LaneType *lane_type_ptr_ = nullptr;
  // 车道线颜色：   WHITE = 0;   YELLOW = 1;   OTHERS = 2;   UNKNOWN = 3;
  legionclaw::interface::LaneLine::LaneColor lane_color_;
  legionclaw::interface::LaneLine::LaneColor *lane_color_ptr_ = nullptr;
  //车道线位置类别：    BOLLARD_LEFT = -5;   FOURTH_LEFT = -4;   THIRD_LEFT =
  //-3;   ADJACENT_LEFT = -2;  //!< lane marking on the left side next to ego
  //lane   EGO_LEFT = -1;       //!< left lane marking of the ego lane EGO_RIGHT
  //= 1;       //!< right lane marking of the ego lane   ADJACENT_RIGHT = 2;
  ////!< lane marking on the right side next to ego lane   THIRD_RIGHT = 3;
  //FOURTH_RIGHT = 4;   BOLLARD_RIGHT = 5;   OTHER = 6;    //!< other types of
  //lane   UNKNOWN = 7;  //!< background
  legionclaw::interface::LaneLine::PosType pos_type_;
  legionclaw::interface::LaneLine::PosType *pos_type_ptr_ = nullptr;
  //车辆坐标系车道线三次曲线系数
  legionclaw::interface::LaneLineCubicCurve curve_vehicle_;
  legionclaw::interface::LaneLineCubicCurve *curve_vehicle_ptr_ = nullptr;
  //图像坐标系车道线三次曲线系数
  legionclaw::interface::LaneLineCubicCurve curve_image_;
  legionclaw::interface::LaneLineCubicCurve *curve_image_ptr_ = nullptr;
  //世界坐标系车道线三次曲线系数
  legionclaw::interface::LaneLineCubicCurve curve_abs_;
  legionclaw::interface::LaneLineCubicCurve *curve_abs_ptr_ = nullptr;
  //车辆坐标系车道线点集
  std::vector<legionclaw::interface::Point3D> pts_vehicle_;
  //图像坐标系车道线点集
  std::vector<legionclaw::interface::Point2D> pts_image_;
  //世界坐标系车道线点集
  std::vector<legionclaw::interface::Point3D> pts_abs_;
  //车道线上顶点与下顶点
  legionclaw::interface::EndPoints image_end_point_;
  legionclaw::interface::EndPoints *image_end_point_ptr_ = nullptr;
  //车道线关键点数组
  std::vector<legionclaw::interface::Point2D> pts_key_;
  //高精车道线id
  uint8_t hd_lane_id_;
  uint8_t *hd_lane_id_ptr_ = nullptr;
  //车道线置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //  车道线质量：   VERY_LOW = 0;   LOW = 1;   HIGH = 2;   VERY_HIGH = 3;
  legionclaw::interface::LaneLine::LaneQuality lane_quality_;
  legionclaw::interface::LaneLine::LaneQuality *lane_quality_ptr_ = nullptr;
  //  融合车道线类别：   CAMERA = 0;   HD = 1;   FUSED = 2;   UNKNOWN = 3;   if
  //  value is 1, lane_quality = 3
  legionclaw::interface::LaneLine::FusedLaneType fused_lane_type_;
  legionclaw::interface::LaneLine::FusedLaneType *fused_lane_type_ptr_ = nullptr;
  //透视变换矩阵
  std::vector<double> homography_mat_;
  //透视变换逆矩阵
  std::vector<double> homography_mat_inv_;
  //  坐标系类别：   FRAME = 0;   IMAGE = 1;   CAMERA = 2;   VEHICLE = 3;
  //  ABSOLUTE = 4;
  legionclaw::interface::LaneLine::LaneCoordinateType lane_coordinate_type_;
  legionclaw::interface::LaneLine::LaneCoordinateType *lane_coordinate_type_ptr_ =
      nullptr;
  //  使用类别：   REAL = 0;   VIRTUAL = 1;
  legionclaw::interface::LaneLine::UseType use_type_;
  legionclaw::interface::LaneLine::UseType *use_type_ptr_ = nullptr;
  //车道线被识别的时间戳
  legionclaw::interface::Time create_time_;
  legionclaw::interface::Time *create_time_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
