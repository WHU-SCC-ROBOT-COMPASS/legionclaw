int32   color     # UNKNOWN = 0; RED = 1; YELLOW = 2; GREEN = 3; BLACK = 4;
uint32   id     # light id
int32   type     # STRAIGHT = 0; TURN_LEFT = 1; TURN_RIGHT = 2; STRAIGHT_TURN_LEFT = 3; STRAIGHT_TURN_RIGHT =4; CIRCULAR = 5; PEDESTRIAN = 6; CYCLIST = 7; UNKNOWN = 8;
float64   confidence     # 置信度confdence: [0-1]
ImageRect   light_rect     # 交通灯矩形框light rect
Point3D   position     # 交通灯位置position of light
float64   distance     # 距离distance between light and stop line
int32[] light_lanes # corresponding lanes of light
float64   tracking_time     # duration of a light since track
bool   blink     # true:
float64   blinking_time     # duration of a light blinking
float64   remaining_time     # V2X time of light
Time   create_time     # time of light detected
