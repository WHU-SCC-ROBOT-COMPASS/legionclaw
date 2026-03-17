Polygon2D[] vehicle_preiew_polygon # 车辆预测多边形
Polygon2D[] obstacles_polygon # 障碍物多边形
PathPoint[] path_points # 轨迹点
float64   lat_error     # 横向误差
float64   lon_error     # 纵向误差
float64   yaw_error     # 角度误差(度)
Point2dList[] obstacles_vec #  
TrajectoryPoint[] warm_start_traj #  
TrajectoryPoint[] smoothed_traj_stage1 #  
TrajectoryPoint[] smoothed_traj_stage2 #  
Trajectory   reference_line     #  
Trajectory[] trajectory_array # 轨迹簇
Trajectory   optimal_coarse_trajectory     #  
Trajectory   optimal_smooth_trajectory     #  
float64   hybrid_a_star_map_time     #  
float64   hybrid_a_star_heuristic_time     #  
float64   hybrid_a_star_rs_time     #  
float64   hybrid_a_star_total_time     #  
float64   ias_collision_avoidance_time     #  
float64   ias_path_smooth_time     #  
float64   ias_speed_smooth_time     #  
float64   ias_total_time     #  
float64   samping_trajectory_time     #  
uint8   is_replan     #  
string   replan_reason     #  
float64   replan_time     #  
int32   replan_num     #  
int32   optimizer_thread_counter     #  
int32   replan_by_context_update_counter     #  
int32   replan_by_large_error_counter     #  
int32   parking_type     #  
int32   moves_counter     # 揉库次数
float64   remain_distance     # 剩余距离
float64   distance_to_leader_obj     # 距离最近障碍物距离
int32   state     #  
