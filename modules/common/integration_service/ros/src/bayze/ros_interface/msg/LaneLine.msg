int32   lane_type     #  车道线类型：   SOLID = 0;   DASHED = 1;   ROAD_DELIMITER = 2;   NONE = 3;   UNKNOWN = 4;
int32   lane_color     #  车道线颜色：   WHITE = 0;   YELLOW = 1;   OTHERS = 2;   UNKNOWN = 3;
int32   pos_type     # 车道线位置类别：    BOLLARD_LEFT = -5;   FOURTH_LEFT = -4;   THIRD_LEFT = -3;   ADJACENT_LEFT = -2;  //!< lane marking on the left side next to ego lane   EGO_LEFT = -1;       //!< left lane marking of the ego lane   EGO_RIGHT = 1;       //!< right lane marking of the ego lane   ADJACENT_RIGHT = 2;  //!< lane marking on the right side next to ego lane   THIRD_RIGHT = 3;   FOURTH_RIGHT = 4;   BOLLARD_RIGHT = 5;   OTHER = 6;    //!< other types of lane   UNKNOWN = 7;  //!< background
LaneLineCubicCurve   curve_vehicle     # 车辆坐标系车道线三次曲线系数
LaneLineCubicCurve   curve_image     # 图像坐标系车道线三次曲线系数
LaneLineCubicCurve   curve_abs     # 世界坐标系车道线三次曲线系数
Point3D[] pts_vehicle # 车辆坐标系车道线点集
Point2D[] pts_image # 图像坐标系车道线点集
Point3D[] pts_abs # 世界坐标系车道线点集
EndPoints   image_end_point     # 车道线上顶点与下顶点
Point2D[] pts_key # 车道线关键点数组
uint8   hd_lane_id     # 高精车道线id
float64   confidence     # 车道线置信度
int32   lane_quality     #   车道线质量：   VERY_LOW = 0;   LOW = 1;   HIGH = 2;   VERY_HIGH = 3;
int32   fused_lane_type     #   融合车道线类别：   CAMERA = 0;   HD = 1;   FUSED = 2;   UNKNOWN = 3;   if value is 1, lane_quality = 3
float64[] homography_mat # 透视变换矩阵
float64[] homography_mat_inv # 透视变换逆矩阵
int32   lane_coordinate_type     #   坐标系类别：   FRAME = 0;   IMAGE = 1;   CAMERA = 2;   VEHICLE = 3;   ABSOLUTE = 4;
int32   use_type     #   使用类别：   REAL = 0;   VIRTUAL = 1;
Time   create_time     # 车道线被识别的时间戳
