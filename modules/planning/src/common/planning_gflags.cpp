#include "planning_gflags.h"

DEFINE_string(planning_config_file, "./conf/planning/planning.json",
              "planning module config file");
DEFINE_string(parking_config_file, "./conf/planning/parking/parking.json",
              "planning module config file");
DEFINE_bool(use_latticegation_mode, true, "use_latticegation_mode");
DEFINE_bool(planning_state_machine_log_enable, true, "planning_state_machine_log_enable");
DEFINE_bool(behavior_lat_state_machine_log_enable, true, "behavior_lat_state_machine_log_enable");
DEFINE_bool(behavior_lon_state_machine_log_enable, true, "behavior_lon_state_machine_log_enable");
DEFINE_bool(parking_state_machine_log_enable, true, "parking_state_machine_log_enable");
DEFINE_string(state_machine_log_dir, "./log/state_machine/",
              "state_machine_log_dir");
DEFINE_string(planning_state_machine_file,
              "./conf/planning/state_machine/planning_state_machine.json",
              "planning_state_machine");
DEFINE_string(planning_state_machine_name, "planning_state_machine",
              "planning_state_machine");
DEFINE_string(behavior_lat_state_machine_file,
              "./conf/planning/state_machine/behavior_lat_state_machine.json",
              "behavior_lat_state_machine");
DEFINE_string(behavior_lat_state_machine_name, "behavior_lat_state_machine",
              "behavior_lat_state_machine");
DEFINE_string(behavior_lon_state_machine_file,
              "./conf/planning/state_machine/behavior_lon_state_machine.json",
              "behavior_lon_state_machine");
DEFINE_string(behavior_lon_state_machine_name, "behavior_lon_state_machine",
              "behavior_lon_state_machine");             
DEFINE_string(parking_state_machine_file,
              "./conf/planning/state_machine/parking_state_machine.json",
              "parking_state_machine");
DEFINE_string(parking_state_machine_name, "parking_state_machine",
              "parking_state_machine");
DEFINE_int32(state_mahine_spin_duration, 10,
             "Loop rate for  state machine spin");
// DEFINE_string(lattice_state_machine_file,
//               "./conf/planning/driving/lattice_planner.json",
//               "state_machine_app_gear_state_machine_file");
DEFINE_string(vehicle_param_file_path,
                "../../common/data/vehicle_param/vehicle_param.json",
                "vehicle_param_file_path");
DEFINE_bool(enable_reference_line_provider_thread, true,
            "Enable reference line provider thread.");

DEFINE_string(lattice_planner_config_file,
              "./conf/planning/driving/lattice_planner.json",
              "lattice planner file path");
