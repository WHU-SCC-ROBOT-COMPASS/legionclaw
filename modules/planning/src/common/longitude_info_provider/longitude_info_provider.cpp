#include "modules/planning/src/common/longitude_info_provider/longitude_info_provider.h"

namespace legionclaw {
namespace planning {

bool LongitudeInfoProvider::ProvideTrajectoryLongitudinalInfo(
    const double &current_velocity, const double &accel,
    const double &limit_speed, std::vector<TrajectoryPoint> &trajectory){
  if (trajectory.size() <= 0) return false;

  double temp_velocity = 0.0;
  double init_mileage = trajectory.front().path_point().s();
  double delta_mileage = 0.0;
  double relative_time = 0.0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    delta_mileage = trajectory.at(i).path_point().s() - init_mileage;
    // target speed^2 of current point.
    temp_velocity =
        current_velocity * current_velocity + 2.0 * accel * delta_mileage;
    trajectory.at(i).set_a(accel);
    if (temp_velocity <= 0.0) {
      temp_velocity = 0.0;
      if (accel <= 0.0)
        trajectory.at(i).set_gear(legionclaw::common::GEAR_PARKING);
      else
        trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);
    } else {
      temp_velocity = sqrt(temp_velocity);
      trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);
      if (accel >= 0.0 && temp_velocity > limit_speed) {
        temp_velocity = limit_speed;
        trajectory.at(i).set_a(0.0);
      }
    }
    trajectory.at(i).set_v(temp_velocity);

    // set time cost
    if (i == 0) {
      trajectory.at(i).set_relative_time(0.0);
    } else {
      if (trajectory.at(i - 1).a() == 0.0) {
        relative_time = trajectory.at(i - 1).relative_time() +
                        (trajectory.at(i).path_point().s() -
                         trajectory.at(i - 1).path_point().s()) /
                            trajectory.at(i - 1).v();
        trajectory.at(i).set_relative_time(relative_time);
      } else {
        relative_time = trajectory.at(i - 1).relative_time() +
                        (trajectory.at(i).v() - trajectory.at(i - 1).v()) /
                            trajectory.at(i - 1).a();
        trajectory.at(i).set_relative_time(relative_time);
      }
    }
  }
  // 轨迹按相对时间采样抽稀
  std::vector<TrajectoryPoint> trajectory_temp;
  double time_adder = 0;
  for(unsigned int j = 0; j < trajectory.size(); ++j) {
    if (trajectory.at(j).relative_time() < 1) {
      trajectory_temp.emplace_back( trajectory.at(j));
      continue;
    }
    time_adder += trajectory.at(j).relative_time() - trajectory.at(j-1).relative_time();
    if (time_adder > 1) {
      trajectory_temp.emplace_back( trajectory.at(j));
      time_adder = 0;
    }
  }

  trajectory = std::move(trajectory_temp);
  return true;
}

}
}