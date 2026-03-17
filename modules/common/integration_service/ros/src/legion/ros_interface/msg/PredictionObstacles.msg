Headers   header     # timestamp is included in header
PredictionObstacle[] prediction_obstacles # make prediction for multiple obstacles
int32   change_origin_flag     # 0:坐标系切换成功 1:坐标系切换中 2:坐标系切换故障
float64   start_timestamp     # start timestamp
float64   end_timestamp     # end timestamp
int32   self_intent     # 自动驾驶车辆意图  0-UNKNOWN, 1-STOP, 2-CRUISE, 3-CHANGE_LANE
int32   scenario     # 场景              0-UNKNOWN, 1000-CRUISE, 1001-CRUISE_URBAN 1002-CRUISE_HIGHWAY, 2000-JUNCTION  2001-JUNCTION_TRAFFIC_LIGHT 2002-JUNCTION_STOP_SIGN
