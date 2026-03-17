#pragma once

#include <gflags/gflags.h>

DECLARE_string(control_config_file);
DECLARE_string(lon_calibration_table_file);
DECLARE_string(stop_distance_calibration_table_file);
DECLARE_string(lqr_calibration_table_file);
DECLARE_int32(state_mahine_spin_duration);
DECLARE_string(state_machine_app_gear_state_machine_file);
DECLARE_string(state_machine_app_epb_state_machine_file);
DECLARE_string(state_machine_log_dir);
DECLARE_bool(state_machine_debug);
DECLARE_bool(trajectory_transform_to_com_drive);
DECLARE_string(vehicle_param_file_path);
// DECLARE_bool(state_transform_to_com_reverse);
// DECLARE_bool(state_transform_to_com_drive);
// DECLARE_bool(use_navigation_mode);