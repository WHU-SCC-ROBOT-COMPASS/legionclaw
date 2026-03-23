#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <optional>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include <modules/localization/src/pose_estimator/kkl/alg/unscented_kalman_filter.hpp>

namespace kkl {
  namespace alg {
    template<typename T, class System>
    class UnscentedKalmanFilterX;
  }
}

namespace legionclaw {
namespace localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief scan matching-based pose estimator using UKF
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration,
               const Eigen::Vector3f& pos,
               const Eigen::Quaternionf& quat,
               double cool_time_duration = 1.0);
  ~PoseEstimator();

  void predict(double stamp);
  void predict(double stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);
  void predict_odom(const Eigen::Matrix4f& odom_delta);
  pcl::PointCloud<PointT>::Ptr correct(double stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  /* getters */
  double last_correction_time() const;
  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;
  Eigen::Vector3f odom_pos() const;
  Eigen::Quaternionf odom_quat() const;
  Eigen::Matrix4f odom_matrix() const;
  const std::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const std::optional<Eigen::Matrix4f>& imu_prediction_error() const;
  const std::optional<Eigen::Matrix4f>& odom_prediction_error() const;

private:
  double init_stamp;
  double prev_stamp;
  double last_correction_stamp;
  double cool_time_duration;

  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;

  Eigen::Matrix4f last_observation;
  std::optional<Eigen::Matrix4f> wo_pred_error;
  std::optional<Eigen::Matrix4f> imu_pred_error;
  std::optional<Eigen::Matrix4f> odom_pred_error;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace localization
}  // namespace legionclaw

#endif  // POSE_ESTIMATOR_HPP
