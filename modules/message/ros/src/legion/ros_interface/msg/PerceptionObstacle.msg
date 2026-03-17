int32   id     #  
Point3D   position     #  
float64   theta     #  
Point3D   velocity     #  
float64   length     #  
float64   width     #  
float64   height     #  
Point3D[] polygon_point #  
float64   tracking_time     #  
int32   type     # 障碍物类型： 0-Unknown 1-Unknown_movable 2-Unknown_unmovable 3-Pedestrian 4-Bicycle   5-Vehicle
int32   lane_position     # 车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE 2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN
float64   confidence     #  
float64   timestamp     #  
uint8   confidence_type     # 置信度类型                  0-CONFIDENCE_UNKNOWN, 1-CONFIDENCE_CN, 2-CONFIDENCE_RAD
Point3D   drops     #  
Point3D   acceleration     #  
Point3D   anchor_point     #  
Point3D[] bounding_box #  
int32   sub_type     #  
float64   height_above_ground     #  
float64[] position_covariance #  
float64[] velocity_covariance #  
float64[] acceleration_covariance #  
uint8   light_status     #  
