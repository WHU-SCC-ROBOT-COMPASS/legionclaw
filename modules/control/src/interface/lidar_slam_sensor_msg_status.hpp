/**
 * @file    example.h
 * @author  editor
 * @date    2020-05-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>

using namespace std;

namespace legionclaw {
namespace localization {
/**
 * @class LidarSlamSensorMsgStatus
 *
 * @brief 消息的描述.
 */
class LidarSlamSensorMsgStatus {
 public:
  LidarSlamSensorMsgStatus() = default;
  ~LidarSlamSensorMsgStatus() = default;

  void set_imu_delay_status(int8_t imu_delay_status) {
    imu_delay_status_ = imu_delay_status;
  }

  int8_t imu_delay_status() const { return imu_delay_status_; }

  inline int8_t *mutable_imu_delay_status() { return &imu_delay_status_; }

  void set_imu_missing_status(int8_t imu_missing_status) {
    imu_missing_status_ = imu_missing_status;
  }

  int8_t imu_missing_status() const { return imu_missing_status_; }

  inline int8_t *mutable_imu_missing_status() { return &imu_missing_status_; }

  void set_imu_data_status(int8_t imu_data_status) {
    imu_data_status_ = imu_data_status;
  }

  int8_t imu_data_status() const { return imu_data_status_; }

  inline int8_t *mutable_imu_data_status() { return &imu_data_status_; }

 protected:
  // IMU_DELAY_NORMAL = 0;
  // IMU_DELAY_1 = 1;
  // IMU_DELAY_2 = 2;
  // IMU_DELAY_3 = 3;
  // IMU_DELAY_ABNORMAL = 4;
  int8_t imu_delay_status_;
  // IMU_MISSING_NORMAL = 0;
  // IMU_MISSING_1 = 1;
  // IMU_MISSING_2 = 2;
  // IMU_MISSING_3 = 3;
  // IMU_MISSING_4 = 4;
  // IMU_MISSING_5 = 5;
  // IMU_MISSING_ABNORMAL = 6;
  int8_t imu_missing_status_;
  // IMU_DATA_NORMAL = 0;
  // IMU_DATA_ABNORMAL = 1;
  // IMU_DATA_OTHER = 2;
  int8_t imu_data_status_;
};
}  // namespace localization
}  // namespace legionclaw
