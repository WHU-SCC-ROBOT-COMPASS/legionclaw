#include "control_gflags.h"

#include <limits>

DEFINE_string(control_config_file, "./conf/control/control.json",
              "control module config file");

DEFINE_string(lon_calibration_table_file,
              "./conf/control/lon_calibration_table.json",
              "lon calibration config file");
// TODO
DEFINE_string(stop_distance_calibration_table_file,
              "./conf/control/stop_distance_calibration_table.json",
              "stop_distance calibration config file");

DEFINE_string(lqr_calibration_table_file,
              "./conf/control/lqr_calibration_table.json",
              "lqr matrix_K calibration config file");

DEFINE_int32(state_mahine_spin_duration, 10,
             "Loop rate for  state machine spin");

DEFINE_string(state_machine_app_gear_state_machine_file,
              "./conf/control/gear_state_machine.json",
              "state_machine_app_gear_state_machine_file");

DEFINE_string(state_machine_app_epb_state_machine_file,
              "./conf/control/epb_state_machine.json",
              "state_machine_app_epb_state_machine_file");

DEFINE_string(state_machine_log_dir, "./log/state_machine/",
              "state_machine_log_dir");

DEFINE_bool(state_machine_debug, true, "state_machine_debug");

DEFINE_bool(
    trajectory_transform_to_com_drive, true,
    "Enable planning trajectory coordinate transformation from center of "
    "rear-axis to center of mass, during forward driving");

DEFINE_string(vehicle_param_file_path,
              "../../common/data/vehicle_param/vehicle_param.json",
              "vehicle param file path");

// DEFINE_bool(state_transform_to_com_reverse,true,"state_transform_to_com_reverse");

// DEFINE_bool(state_transform_to_com_drive,true,"state_transform_to_com_drive");

// DEFINE_bool(use_navigation_mode,false,"use_navigation_mode");