int32   obstacle_id     # Obstacle ID
Point3D   polygon_point     # 障碍物边框点列
Point3D   position     #  
Point3D   front_position     #  
Point3D   velocity     #  
Point3D   raw_velocity     # from perception
Point3D   acceleration     #  
float64   velocity_heading     #  
float64   speed     #  
float64   acc     #  
float64   theta     #  
float64   length     #  
float64   width     #  
float64   height     #  
float64   tracking_time     #  
float64   timestamp     #  
Point3D   t_position     #  
Point3D   t_velocity     # [deprecated = true]
float64   t_velocity_heading     # [deprecated = true]
float64   t_speed     # [deprecated = true]
Point3D   t_acceleration     # [deprecated = true]
float64   t_acc     # [deprecated = true]
bool   is_still     # [deprecated = false]
int32   type     # 感知到的障碍物类型         0-UNKNOWN, 1-UNKNOWN_MOVABLE, 2-UNKNOWN_UNMOVABLE, 3-PEDESTRIAN, 4-BICYCLE,     5-VEHICLE
float64   label_update_time_delta     #  
ObstaclePriority   priority     # CAUTION = 1; NORMAL = 2; IGNORE = 3;
bool   is_near_junction     #  
PredictionTrajectoryPoint[] future_trajectory_points # Obstacle ground-truth labels:
TrajectoryPointInPrediction[] short_term_predicted_trajectory_points # Obstacle short-term predicted trajectory points
TrajectoryInPrediction[] predicted_trajectory # Obstacle predicted trajectories
TrajectoryPointInPrediction[] adc_trajectory_point # ADC trajectory at the same frame
