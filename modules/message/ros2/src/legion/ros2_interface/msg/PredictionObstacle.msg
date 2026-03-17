PerceptionObstacle   perception_obstacle     # perception info of obstacle
float64   timestamp     # GPS time in seconds
float64   predicted_period     # the length of the time for this prediction (e.g. 10s)
TrajectoryInPrediction[] trajectory # can have multiple trajectories per obstacle
ObstacleIntent   intent     # 估计障碍物的意图        0-UNKNOWN, 1-STOP, 2-STATIONARY, 3-移动 MOVING, 4-HANGE_LANE, 5-LOW_ACCELERATION, 6-HIGH_ACCELERATION, 7-LOW_DECELERATION, 8-HIGH_DECELERATION, 
ObstaclePriority   priority     # 障碍物优先级               1-CAUTION, 2-NORMAL, 3-IGNORE
ObstacleInteractiveTag   interactive_tag     #  
bool   is_static     # is obstacle static (default = false)
