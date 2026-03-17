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
 * @class LidarSlamStatus
 *
 * @brief 消息的描述.
 */
class LidarSlamStatus {
 public:
  LidarSlamStatus() = default;
  ~LidarSlamStatus() = default;

  void set_local_lidar_consistency(int8_t local_lidar_consistency) {
    local_lidar_consistency_ = local_lidar_consistency;
  }

  int8_t local_lidar_consistency() const { return local_lidar_consistency_; }

  inline int8_t *mutable_local_lidar_consistency() {
    return &local_lidar_consistency_;
  }

  void set_gnss_consistency(int8_t gnss_consistency) {
    gnss_consistency_ = gnss_consistency;
  }

  int8_t gnss_consistency() const { return gnss_consistency_; }

  inline int8_t *mutable_gnss_consistency() { return &gnss_consistency_; }

  void set_LocalLidarStatus(int8_t LocalLidarStatus) {
    LocalLidarStatus_ = LocalLidarStatus;
  }

  int8_t LocalLidarStatus() const { return LocalLidarStatus_; }

  inline int8_t *mutable_LocalLidarStatus() { return &LocalLidarStatus_; }

  void set_local_lidar_quality(int8_t local_lidar_quality) {
    local_lidar_quality_ = local_lidar_quality;
  }

  int8_t local_lidar_quality() const { return local_lidar_quality_; }

  inline int8_t *mutable_local_lidar_quality() { return &local_lidar_quality_; }

  void set_gnsspos_position_type(int8_t gnsspos_position_type) {
    gnsspos_position_type_ = gnsspos_position_type;
  }

  int8_t gnsspos_position_type() const { return gnsspos_position_type_; }

  inline int8_t *mutable_gnsspos_position_type() {
    return &gnsspos_position_type_;
  }

  void set_running_status(int8_t running_status) {
    running_status_ = running_status;
  }

  int8_t running_status() const { return running_status_; }

  inline int8_t *mutable_running_status() { return &running_status_; }

 protected:
  // LS_LOCAL_LIDAR_CONSISTENCY_00 = 0;
  // The difference is less than threshold 1
  // LS_LOCAL_LIDAR_CONSISTENCY_01 = 1;
  // The difference is bigger than threshold 1 but less than threshold 2
  // LS_LOCAL_LIDAR_CONSISTENCY_02 = 2;
  // The difference is bigger than threshold 2
  // LS_LOCAL_LIDAR_CONSISTENCY_03 = 3;
  // others
  int8_t local_lidar_consistency_;
  // LS_GNSS_CONSISTENCY_00 = 0;
  // The difference is less than threshold 1
  // LS_GNSS_CONSISTENCY_01 = 1;
  // The difference is bigger than threshold 1 but less than threshold 2
  // LS_GNSS_CONSISTENCY_02 = 2;
  // The difference is bigger than threshold 2
  // LS_GNSS_CONSISTENCY_03 = 3;
  // others
  int8_t gnss_consistency_;
  // LS_LOCAL_LIDAR_NORMAL = 0;
  // Localization result satisfy threshold
  // LS_LOCAL_LIDAR_MAP_MISSING = 1;
  // Can't find localization map (config.xml)
  // LS_LOCAL_LIDAR_EXTRINSICS_MISSING = 2;
  // Missing extrinsic parameters
  // LS_LOCAL_LIDAR_MAP_LOADING_FAILED = 3;
  // Fail to load localization map
  // LS_LOCAL_LIDAR_NO_OUTPUT = 4;
  // No output (comparing to timestamp of imu msg)
  // LS_LOCAL_LIDAR_OUT_OF_MAP = 5;
  // Coverage of online pointcloud and map is lower than threshold
  // LS_LOCAL_LIDAR_NOT_GOOD = 6;
  // Localization result do not meet threshold
  // LS_LOCAL_LIDAR_UNDEFINED_STATUS = 7;
  // others
  int8_t LocalLidarStatus_;
  // LS_LOCAL_LIDAR_VERY_GOOD = 0;
  // LS_LOCAL_LIDAR_GOOD = 1;
  // LS_LOCAL_LIDAR_NOT_BAD = 2;
  // LS_LOCAL_LIDAR_BAD = 3;
  int8_t local_lidar_quality_;
  // NONE = 0;
  // No solution
  // FIXEDPOS = 1;
  // Position has been fixed by the FIX POSITION command or by position
  // averaging FIXEDHEIGHT = 2; Position has been fixed by the FIX HEIGHT, or
  // FIX AUTO, command or by position averaging FLOATCONV = 4; Solution from
  // floating point carrier phase anbiguities WIDELANE = 5; Solution from
  // wide-lane ambiguities NARROWLANE = 6; Solution from narrow-lane ambiguities
  // DOPPLER_VELOCITY = 8;
  // Velocity computed using instantaneous Doppler
  // SINGLE = 16;
  // Single point position
  // PSRDIFF = 17;
  // Pseudorange differential solution
  // WAAS = 18;
  // Solution calculated using corrections from an SBAS
  // PROPOGATED = 19;
  // Propagated by a Kalman filter without new observations
  // OMNISTAR = 20;
  // OmniSTAR VBS position
  // L1_FLOAT = 32;
  // Floating L1 albiguity solution
  // IONOFREE_FLOAT = 33;
  // Floating ionospheric free ambiguity solution
  // NARROW_FLOAT = 34;
  // Floating narrow-lane anbiguity solution
  // L1_INT = 48;
  // Integer L1 ambiguity solution
  // WIDE_INT = 49;
  // Integer wide-lane ambiguity solution
  // NARROW_INT = 50;
  // Integer narrow-lane ambiguity solution
  // RTK_DIRECT_INS = 51;
  // RTK status where RTK filter is directly initialized from the INS filter
  // INS_SBAS = 52;
  // INS calculated position corrected for the antenna
  // INS_PSRSP = 53;
  // INS pseudorange single point solution - no DGPS corrections
  // INS_PSRDIFF = 54;
  // INS pseudorange differential solution
  // INS_RTKFLOAT = 55;
  // INS RTK float point ambiguities solution
  // INS_RTKFIXED = 56;
  // INS RTK fixed ambiguities solution
  // INS_OMNISTAR = 57;
  // INS OmniSTAR VBS solution
  // INS_OMNISTAR_HP = 58;
  // INS OmniSTAR high precision solution
  // INS_OMNISTAR_XP = 59;
  // INS OmniSTAR extra precision solution
  // OMNISTAR_HP = 64;
  // OmniSTAR high precision
  // OMNISTAR_XP = 65;
  // OmniSTAR extra precision
  // PPP_CONVERGING = 68;
  // Precise Point Position(PPP) solution converging
  // PPP = 69;
  // Precise Point Position(PPP)solution
  // INS_PPP_Converging = 73;
  // INS NovAtel CORRECT Precise Point Position(PPP) solution converging
  // INS_PPP = 74;
  // INS NovAtel CORRECT Precise Point Position(PPP) solution
  // MSG_LOSS = 91;
  // Gnss position message loss
  int8_t gnsspos_position_type_;
  // LS_SOL_LIDAR_GNSS = 0;
  // LS_SOL_X_GNSS = 1;
  // LS_SOL_LIDAR_X = 2;
  // LS_SOL_LIDAR_XX = 3;
  // LS_SOL_LIDAR_XXX = 4;
  // LS_SOL_X_X = 5;
  // LS_SOL_X_XX = 6;
  // LS_SOL_X_XXX = 7;
  // LS_SSOL_LIDAR_GNSS = 8;
  // LS_SSOL_X_GNSS = 9;
  // LS_SSOL_LIDAR_X = 10;
  // LS_SSOL_LIDAR_XX = 11;
  // LS_SSOL_LIDAR_XXX = 12;
  // LS_SSOL_X_X = 13;
  // LS_SSOL_X_XX = 14;
  // LS_SSOL_X_XXX = 15;
  // LS_NOSOL_LIDAR_GNSS = 16;
  // LS_NOSOL_X_GNSS = 17;
  // LS_NOSOL_LIDAR_X = 18;
  // LS_NOSOL_LIDAR_XX = 19;
  // LS_NOSOL_LIDAR_XXX = 20;
  // LS_NOSOL_X_X = 21;
  // LS_NOSOL_X_XX = 22;
  // LS_NOSOL_X_XXX = 23;
  // LS_RUNNING_INIT = 24;
  int8_t running_status_;
};
}  // namespace localization
}  // namespace legionclaw
