#pragma once

#include <gflags/gflags.h>

DECLARE_string(planning_config_file);
DECLARE_string(parking_config_file);
DECLARE_bool(use_latticegation_mode);
DECLARE_bool(planning_state_machine_log_enable);
DECLARE_bool(behavior_lat_state_machine_log_enable);
DECLARE_bool(behavior_lon_state_machine_log_enable);
DECLARE_bool(parking_state_machine_log_enable);
DECLARE_string(state_machine_log_dir);
DECLARE_string(planning_state_machine_file);
DECLARE_string(planning_state_machine_name);
DECLARE_string(behavior_lat_state_machine_file);
DECLARE_string(behavior_lat_state_machine_name);
DECLARE_string(behavior_lon_state_machine_file);
DECLARE_string(behavior_lon_state_machine_name);
DECLARE_string(parking_state_machine_file);
DECLARE_string(parking_state_machine_name);
DECLARE_int32(state_mahine_spin_duration);
// DECLARE_string(lattice_state_machine_file);
DECLARE_string(vehicle_param_file_path);

DECLARE_string(lattice_planner_config_file);

// parameter for reference line
DECLARE_bool(enable_reference_line_provider_thread);

