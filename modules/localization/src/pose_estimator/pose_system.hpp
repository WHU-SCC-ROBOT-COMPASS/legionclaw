#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP
#define G_MS2 9.80665f

#include <modules/localization/src/pose_estimator/kkl/alg/unscented_kalman_filter.hpp>

namespace legionclaw {
namespace localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;

public:
  PoseSystem() { dt = 0.01; }

  // system equation (without input)
  VectorXt f(const VectorXt& state) const {
    VectorXt next_state(16);
    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();
    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    next_state.middleRows(0, 3) = pt + vt * dt;  // position
    next_state.middleRows(3, 3) = vt;              // velocity
    Quaterniont qt_ = qt;
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity
    return next_state;
  }

  // system equation with IMU input
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);
    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();
    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);
    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);

    next_state.middleRows(0, 3) = pt + vt * dt;
    Vector3t g(0.0f, 0.0f, 9.80665f);
    Vector3t acc_ = raw_acc - acc_bias;
    Vector3t acc = qt * acc_;
    // IMU mounted inverted - acceleration unit is G
    next_state.middleRows(3, 3) = vt + (acc * G_MS2 - g) * dt;

    Vector3t gyro = raw_gyro - gyro_bias;
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);
    next_state.middleRows(13, 3) = state.middleRows(13, 3);
    return next_state;
  }

  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();
    return observation;
  }

  double dt;
};

}  // namespace localization
}  // namespace legionclaw

#endif  // POSE_SYSTEM_HPP
