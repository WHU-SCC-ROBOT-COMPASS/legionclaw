/**
 * @file    gaussian_info.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class GaussianInfo {
public:
  GaussianInfo() {
    sigma_x_ = 0.0;
    sigma_y_ = 0.0;
    correlation_ = 0.0;
    area_probability_ = 0.0;
    ellipse_a_ = 0.0;
    ellipse_b_ = 0.0;
    theta_a_ = 0.0;
  }
  ~GaussianInfo() = default;

  inline void set_sigma_x(const double &sigma_x) {
    sigma_x_ = sigma_x;
    sigma_x_ptr_ = &sigma_x_;
  }

  inline const double &sigma_x() const { return sigma_x_; }

  inline double *mutable_sigma_x() { return &sigma_x_; }

  inline bool has_sigma_x() { return (sigma_x_ptr_ != nullptr); }

  inline void set_sigma_y(const double &sigma_y) {
    sigma_y_ = sigma_y;
    sigma_y_ptr_ = &sigma_y_;
  }

  inline const double &sigma_y() const { return sigma_y_; }

  inline double *mutable_sigma_y() { return &sigma_y_; }

  inline bool has_sigma_y() { return (sigma_y_ptr_ != nullptr); }

  inline void set_correlation(const double &correlation) {
    correlation_ = correlation;
    correlation_ptr_ = &correlation_;
  }

  inline const double &correlation() const { return correlation_; }

  inline double *mutable_correlation() { return &correlation_; }

  inline bool has_correlation() { return (correlation_ptr_ != nullptr); }

  inline void set_area_probability(const double &area_probability) {
    area_probability_ = area_probability;
    area_probability_ptr_ = &area_probability_;
  }

  inline const double &area_probability() const { return area_probability_; }

  inline double *mutable_area_probability() { return &area_probability_; }

  inline bool has_area_probability() {
    return (area_probability_ptr_ != nullptr);
  }

  inline void set_ellipse_a(const double &ellipse_a) {
    ellipse_a_ = ellipse_a;
    ellipse_a_ptr_ = &ellipse_a_;
  }

  inline const double &ellipse_a() const { return ellipse_a_; }

  inline double *mutable_ellipse_a() { return &ellipse_a_; }

  inline bool has_ellipse_a() { return (ellipse_a_ptr_ != nullptr); }

  inline void set_ellipse_b(const double &ellipse_b) {
    ellipse_b_ = ellipse_b;
    ellipse_b_ptr_ = &ellipse_b_;
  }

  inline const double &ellipse_b() const { return ellipse_b_; }

  inline double *mutable_ellipse_b() { return &ellipse_b_; }

  inline bool has_ellipse_b() { return (ellipse_b_ptr_ != nullptr); }

  inline void set_theta_a(const double &theta_a) {
    theta_a_ = theta_a;
    theta_a_ptr_ = &theta_a_;
  }

  inline const double &theta_a() const { return theta_a_; }

  inline double *mutable_theta_a() { return &theta_a_; }

  inline bool has_theta_a() { return (theta_a_ptr_ != nullptr); }

  void operator=(const GaussianInfo &gaussian_info) { CopyFrom(gaussian_info); }

  void CopyFrom(const GaussianInfo &gaussian_info) {
    sigma_x_ = gaussian_info.sigma_x();
    sigma_y_ = gaussian_info.sigma_y();
    correlation_ = gaussian_info.correlation();
    area_probability_ = gaussian_info.area_probability();
    ellipse_a_ = gaussian_info.ellipse_a();
    ellipse_b_ = gaussian_info.ellipse_b();
    theta_a_ = gaussian_info.theta_a();
  }

protected:
  //标准差x
  double sigma_x_;
  double *sigma_x_ptr_ = nullptr;
  //标准差y
  double sigma_y_;
  double *sigma_y_ptr_ = nullptr;
  //相关性
  double correlation_;
  double *correlation_ptr_ = nullptr;
  // Information of representative uncertainty area
  double area_probability_;
  double *area_probability_ptr_ = nullptr;
  //椭圆长轴
  double ellipse_a_;
  double *ellipse_a_ptr_ = nullptr;
  //椭圆短轴
  double ellipse_b_;
  double *ellipse_b_ptr_ = nullptr;
  //椭圆朝向
  double theta_a_;
  double *theta_a_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
