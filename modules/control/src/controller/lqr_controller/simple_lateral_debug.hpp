/**
 * @file simple_lateral_debug.hpp
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2018-07-07
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#ifndef CONTROLLER_SIMPLE_LATERAL_DEBUG_H_
#define CONTROLLER_SIMPLE_LATERAL_DEBUG_H_
#include "modules/common/interface/trajectory_point.hpp"
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

/**
 * @class SimpleLateralDebug
 *
 * @brief SimpleLateralDebug.
 */
class SimpleLateralDebug {
 public:
  SimpleLateralDebug() = default;
  ~SimpleLateralDebug() = default;
  void set_lateral_error(double lateral_error) {
    lateral_error_ = lateral_error;
  }

  double lateral_error() const { return lateral_error_; }

  void set_longitudinal_error(double longitudinal_error) {
    longitudinal_error_ = longitudinal_error;
  }

  double longitudinal_error() const { return longitudinal_error_; }

  void set_ref_heading(double ref_heading) { ref_heading_ = ref_heading; }

  double ref_heading() const { return ref_heading_; }

  void set_heading(double heading) { heading_ = heading; }

  double heading() const { return heading_; }

  void set_heading_error(double heading_error) {
    heading_error_ = heading_error;
  }

  double heading_error() const { return heading_error_; }

  void set_heading_error_rate(double heading_error_rate) {
    heading_error_rate_ = heading_error_rate;
  }

  double heading_error_rate() const { return heading_error_rate_; }

  void set_lateral_error_rate(double lateral_error_rate) {
    lateral_error_rate_ = lateral_error_rate;
  }

  double lateral_error_rate() const { return lateral_error_rate_; }

  void set_curvature(double curvature) { curvature_ = curvature; }

  double curvature() const { return curvature_; }

  void set_front_steer_angle(double steer_angle) {
    front_steer_angle_ = steer_angle;
  }

  double front_steer_angle() const { return front_steer_angle_; }

  void set_rear_steer_angle(double steer_angle) {
    rear_steer_angle_ = steer_angle;
  }

  double rear_steer_angle() const { return rear_steer_angle_; }

  void set_front_steer_angle_feedforward(double steer_angle_feedforward) {
    front_steer_angle_feedforward_ = steer_angle_feedforward;
  }

  double front_steer_angle_feedforward() const {
    return front_steer_angle_feedforward_;
  }

  void set_rear_steer_angle_feedforward(double steer_angle_feedforward) {
    rear_steer_angle_feedforward_ = steer_angle_feedforward;
  }

  double rear_steer_angle_feedforward() const {
    return rear_steer_angle_feedforward_;
  }

  void set_front_steer_angle_lateral_contribution(
      double steer_angle_lateral_contribution) {
    front_steer_angle_lateral_contribution_ = steer_angle_lateral_contribution;
  }

  double front_steer_angle_lateral_contribution() const {
    return front_steer_angle_lateral_contribution_;
  }

  void set_front_steer_angle_lateral_rate_contribution(
      double steer_angle_lateral_rate_contribution) {
    front_steer_angle_lateral_rate_contribution_ =
        steer_angle_lateral_rate_contribution;
  }

  double front_steer_angle_lateral_rate_contribution() const {
    return front_steer_angle_lateral_rate_contribution_;
  }

  void set_front_steer_angle_heading_contribution(
      double steer_angle_heading_contribution) {
    front_steer_angle_heading_contribution_ = steer_angle_heading_contribution;
  }

  double front_steer_angle_heading_contribution() const {
    return front_steer_angle_heading_contribution_;
  }

  void set_front_steer_angle_heading_rate_contribution(
      double steer_angle_heading_rate_contribution) {
    front_steer_angle_heading_rate_contribution_ =
        steer_angle_heading_rate_contribution;
  }

  double front_steer_angle_heading_rate_contribution() const {
    return front_steer_angle_heading_rate_contribution_;
  }

  void set_rear_steer_angle_lateral_contribution(
      double steer_angle_lateral_contribution) {
    rear_steer_angle_lateral_contribution_ = steer_angle_lateral_contribution;
  }

  double rear_steer_angle_lateral_contribution() const {
    return rear_steer_angle_lateral_contribution_;
  }

  void set_rear_steer_angle_lateral_rate_contribution(
      double steer_angle_lateral_rate_contribution) {
    rear_steer_angle_lateral_rate_contribution_ =
        steer_angle_lateral_rate_contribution;
  }

  double rear_steer_angle_lateral_rate_contribution() const {
    return rear_steer_angle_lateral_rate_contribution_;
  }

  void set_rear_steer_angle_heading_contribution(
      double steer_angle_heading_contribution) {
    rear_steer_angle_heading_contribution_ = steer_angle_heading_contribution;
  }

  double rear_steer_angle_heading_contribution() const {
    return rear_steer_angle_heading_contribution_;
  }

  void set_rear_steer_angle_heading_rate_contribution(
      double steer_angle_heading_rate_contribution) {
    rear_steer_angle_heading_rate_contribution_ =
        steer_angle_heading_rate_contribution;
  }

  double rear_steer_angle_heading_rate_contribution() const {
    return rear_steer_angle_heading_rate_contribution_;
  }

  void set_front_steer_angle_feedback(double steer_angle_feedback) {
    front_steer_angle_feedback_ = steer_angle_feedback;
  }

  double front_steer_angle_feedback() const {
    return front_steer_angle_feedback_;
  }

  void set_rear_steer_angle_feedback(double steer_angle_feedback) {
    rear_steer_angle_feedback_ = steer_angle_feedback;
  }

  double rear_steer_angle_feedback() const {
    return rear_steer_angle_feedback_;
  }

  void set_front_steering_position(double steering_position) {
    front_steering_position_ = steering_position;
  }

  double front_steering_position() const { return front_steering_position_; }

  void set_rear_steering_position(double steering_position) {
    rear_steering_position_ = steering_position;
  }

  double rear_steering_position() const { return rear_steering_position_; }

  void set_ref_speed(double ref_speed) { ref_speed_ = ref_speed; }

  double ref_speed() const { return ref_speed_; }

  void set_front_steer_angle_limited(double steer_angle_limited) {
    front_steer_angle_limited_ = steer_angle_limited;
  }

  double front_steer_angle_limited() const {
    return front_steer_angle_limited_;
  }

  void set_rear_steer_angle_limited(double steer_angle_limited) {
    rear_steer_angle_limited_ = steer_angle_limited;
  }

  double rear_steer_angle_limited() const { return rear_steer_angle_limited_; }

  void set_lateral_acceleration(double lateral_acceleration) {
    lateral_acceleration_ = lateral_acceleration;
  }

  double lateral_acceleration() const { return lateral_acceleration_; }

  void set_lateral_jerk(double lateral_jerk) { lateral_jerk_ = lateral_jerk; }

  double lateral_jerk() const { return lateral_jerk_; }

  void set_ref_heading_rate(double ref_heading_rate) {
    ref_heading_rate_ = ref_heading_rate;
  }

  double ref_heading_rate() const { return ref_heading_rate_; }

  void set_heading_rate(double heading_rate) { heading_rate_ = heading_rate; }

  double heading_rate() const { return heading_rate_; }

  void set_ref_heading_acceleration(double ref_heading_acceleration) {
    ref_heading_acceleration_ = ref_heading_acceleration;
  }

  double ref_heading_acceleration() const { return ref_heading_acceleration_; }

  void set_heading_acceleration(double heading_acceleration) {
    heading_acceleration_ = heading_acceleration;
  }

  double heading_acceleration() const { return heading_acceleration_; }

  void set_heading_error_acceleration(double heading_error_acceleration) {
    heading_error_acceleration_ = heading_error_acceleration;
  }

  double heading_error_acceleration() const {
    return heading_error_acceleration_;
  }

  void set_ref_heading_jerk(double ref_heading_jerk) {
    ref_heading_jerk_ = ref_heading_jerk;
  }

  double ref_heading_jerk() const { return ref_heading_jerk_; }

  void set_heading_jerk(double heading_jerk) { heading_jerk_ = heading_jerk; }

  double heading_jerk() const { return heading_jerk_; }

  void set_heading_error_jerk(double heading_error_jerk) {
    heading_error_jerk_ = heading_error_jerk;
  }

  double heading_error_jerk() const { return heading_error_jerk_; }

  void set_lateral_error_feedback(double lateral_error_feedback) {
    lateral_error_feedback_ = lateral_error_feedback;
  }

  double lateral_error_feedback() const { return lateral_error_feedback_; }

  void set_heading_error_feedback(double heading_error_feedback) {
    heading_error_feedback_ = heading_error_feedback;
  }

  double heading_error_feedback() const { return heading_error_feedback_; }

  void set_current_target_point(
      legionclaw::interface::TrajectoryPoint current_target_point) {
    current_target_point_ = current_target_point;
  }

  legionclaw::interface::TrajectoryPoint current_target_point() const {
    return current_target_point_;
  }

  void set_front_steer_angle_feedback_augment(
      double steer_angle_feedback_augment) {
    front_steer_angle_feedback_augment_ = steer_angle_feedback_augment;
  }

  double front_steer_angle_feedback_augment() const {
    return front_steer_angle_feedback_augment_;
  }

  void set_rear_steer_angle_feedback_augment(
      double steer_angle_feedback_augment) {
    rear_steer_angle_feedback_augment_ = steer_angle_feedback_augment;
  }

  double rear_steer_angle_feedback_augment() const {
    return rear_steer_angle_feedback_augment_;
  }

 protected:
  double lateral_error_;
  double longitudinal_error_;
  double ref_heading_;
  double heading_;
  double heading_error_;
  double heading_error_rate_;
  double lateral_error_rate_;
  double curvature_;
  double front_steer_angle_;
  double rear_steer_angle_;
  double front_steer_angle_feedforward_;
  double rear_steer_angle_feedforward_;
  double front_steer_angle_lateral_contribution_;
  double front_steer_angle_lateral_rate_contribution_;
  double front_steer_angle_heading_contribution_;
  double front_steer_angle_heading_rate_contribution_;
  double rear_steer_angle_lateral_contribution_;
  double rear_steer_angle_lateral_rate_contribution_;
  double rear_steer_angle_heading_contribution_;
  double rear_steer_angle_heading_rate_contribution_;
  double front_steer_angle_feedback_;
  double rear_steer_angle_feedback_;
  double front_steering_position_;
  double rear_steering_position_;
  double ref_speed_;
  double front_steer_angle_limited_;
  double rear_steer_angle_limited_;

  double lateral_acceleration_;
  // second time derivative of lateral error rate, in m/s^3
  double lateral_jerk_;

  double ref_heading_rate_;
  double heading_rate_;

  // heading_acceleration, as known as yaw acceleration, is the time derivative
  // of heading rate,  in rad/s^2
  double ref_heading_acceleration_;
  double heading_acceleration_;
  double heading_error_acceleration_;

  // heading_jerk, as known as yaw jerk, is the second time derivative of
  // heading rate, in rad/s^3
  double ref_heading_jerk_;
  double heading_jerk_;
  double heading_error_jerk_;

  // modified lateral_error and heading_error with look-ahead or look-back
  // station, as the feedback term for control usage
  double lateral_error_feedback_;
  double heading_error_feedback_;

  // current planning target point
  legionclaw::interface::TrajectoryPoint current_target_point_;

  // Augmented feedback control term in addition to LQR control
  double front_steer_angle_feedback_augment_;
  double rear_steer_angle_feedback_augment_;
};
}  // namespace control
}  // namespace legionclaw
#endif
