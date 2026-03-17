#ifndef _LANELET2_EX_H_
#define _LANELET2_EX_H_
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/GeometryHelper.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/RegulatoryElement.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/CompoundLineString.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_core/utility/CompoundIterator.h>
#include <lanelet2_core/utility/HybridMap.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_core/utility/ReverseAndForwardIterator.h>
#include <lanelet2_core/utility/TransformIterator.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <modules/common/interface/point_3d.hpp>
#include <math.h>
#define _MIN_DIFF_ANGLE 30.0

namespace lanelet2_ex
{
   //轨迹方向枚举
   enum Trajectory_dir
   {
      STRAIGHT = 0,
      LEFT = 1,
      RIGHT = 2,
   };

   //get route产生轨迹的属性
   struct Lan_Atr
   {
      Trajectory_dir dir; //方向
      double dis;         //距离
      double theta_diff;
      int index;          //matched index
   };

   /**
     * @brief 找到map中包含point的lanelets（包含，指点在边界框box中）
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标,没有z坐标]
     * @return lanelet的id集合
     */
   lanelet::Ids GetClosestLanelets(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point);

   /**
     * @brief 找到map中距离point范围range内的lanelets（点point到车道边界框距离在范围内）
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标,没有z坐标]
     * @param range [距离阈值]
     * @return lanelet的id集合
     */
   lanelet::Ids GetClosestLaneletsInRange(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double range);

   /**
     * @brief 找到map中包含point的lanelets其中心线中距离point最近的点（包含，指点在边界框box中）和该lanelet
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param bDirectionBased [是否考虑航向角]
     * @param cpoints [output：点的集合]
     * @param ids [output：lanelet的id集合]
     * @return 点的集合size（lanelet的集合size）
     */
   int GetClosestPoints_Lanelets(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, 
                                 lanelet::ConstPoints3d &cpoints, lanelet::Ids &ids, bool &in_lane);

    /**
       * @brief 找到map中包含point的lanelets其中心线中距离point最近的点（包含，指点在边界框box中）和该lanelet
       * @param map [地图]
       * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
       * @param theta [航向，正北为0，顺时针增加]
       * @param bDirectionBased [是否考虑航向角]
       * @param cpoints [output：点的集合]
       * @param ids [output：lanelet的id集合]
       * @return 点的集合size（lanelet的集合size）
       */
    int GetClosestPoints_Lanelets(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, 
                                  lanelet::ConstPoints3d &cpoints, lanelet::Ids &ids);

   /**
     * @brief 找到距离点point范围range内的lanelets中心线上距离point最近的点(include theta)
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param range [距离搜索阈值]
     * @param path_point [最近的点(x,y,theta)]
     * @return bool
     */
   bool GetClosestPathPointInRange(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point,
                                   const double theta, const double range,
                                   std::pair<lanelet::ConstPoint3d, double> &path_point);
   /**
     * @brief 找到距离点point范围range内的lanelets 和中心线上距离point最近的点
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param bDirectionBased [是否考虑航向角]
     * @param cpoints [output：点的集合]
     * @param ids [output：lanelet的id集合]
     * @param range [距离搜索阈值]
     * @return 点的集合size（lanelet的集合size）
     */
   int GetClosestPoints_LaneletsInRange(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, lanelet::ConstPoints3d &cpoints, lanelet::Ids &ids, const double range);

   /**
     * @brief 找到距离点point范围range内的最近的车道，和中心线上距离最近的点(距离考虑物理距离和角度偏差)
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param range [距离搜索阈值]
     * @param weight_times [角度偏差所占权重的计算倍数]
     * @return 最近的中心线上的点，和该车道id的pair
     */
   std::pair<lanelet::ConstPoint3d, lanelet::Id> GetClosestPoint_LaneletInRange(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const double range, const double weight_times);

   /**
     * @brief 找到map中距离point最近的lanelet（中心线最近）
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标,没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param bDirectionBased [是否考虑航向角]
     * @return 最近的中心线上的点，和该车道id
     */
   std::pair<lanelet::ConstPoint3d, lanelet::Id> LocatePointOnCenterLine(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased);

   /**
     * @brief 在搜索范围内（到车道边界框距离）找到map中距离point最近的lanelet（中心线最近）
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标,没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param bDirectionBased [是否考虑航向角]
     * @param range[距离车道的距离搜索阈值（到车道边界框的距离）]
     * @return 最近的中心线上的点，和该车道id
     */
   std::pair<lanelet::ConstPoint3d, lanelet::Id> LocatePointOnCenterLineInRange(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, const double range);

   /**
     * @brief 定位点到单个车道，并计算到左右边线距离
     * @param map [地图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标,没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @return pair<left_distance,right_distance>到左右边线的距离
     */
   std::pair<double, double> GetLaneBoundDistance(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta);

   /**
     * @brief 找到line中距离point最近的segment（一个segment是两个顺序点）
     * @param point  [输入点]
     * @param line [输入线]
     * @return segment [最近的segment]
     */
   lanelet::Segment3d FIndCLoseSegment(const lanelet::Point3d point, lanelet::LineString3d line);

   /**
     * @brief 定位点到单个车道，并根据到左右车道距离，调整定位点的位置
     * @param map [地图]
     * @param point [坐标，目前传入ENU坐标,没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param distances [距离左右车道线距离，first为左，second为右]
     * @param fixdir [修正依据左边线1，或者右边线2,只能选择一种]
     * @return lanelet::Point3D [经过横向修正过的点坐标]
     */
   lanelet::Point3d FixLocationPoint(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, std::pair<double, double> distances, lanelet2_ex::Trajectory_dir fixdir);

   /*
    std::vector<std::pair<lanelet::Points3d,lanelet2_ex::Trajectory_dir>> GetTrajectorys(const lanelet::LaneletMapUPtr map, const lanelet::routing::RoutingGraphUPtr routing_map, const int level, const lanelet::Point3d point, const double theta, const bool bDirectionBased, const std::set<lanelet::Ids> besides_lanelet, const double max_distance);
    */
   /**
    * @brief 在给定的线中，找到距离给定点point最近的点的索引index
    * @param point  给定的点
    * @param cline 给定的线
    * @return 该点在给定线上的索引，最近点在线上的索引
    */
   unsigned int GetPointIndex(const lanelet::ConstPoint3d point, const lanelet::ConstLineString3d cline);

   /**
    * @brief 在给定的线中，找到距离给定点point最近的点的Lan_Atr
    * @param point  给定的点
    * @param cline 给定的线
    * @return Lan_Atr
    */
    Lan_Atr GetLanAtr(const lanelet::ConstPoint3d point, const double angle, const lanelet::ConstLineString3d cline, const Trajectory_dir dir);

   /**
     * @brief 将给定的线（path）插入到paths前，即对于给定的一系列路径，在其前面都加入一段路径
     * @param path 前置统一的路径
     * @param paths 待插入的所有路径[input][output]
     * @return 所有路径的数量
     */
   int LinkPath(lanelet::ConstPoints3d path, std::vector<lanelet::ConstPoints3d> &paths);

   /**
     * @brief 将给定的lanelet插入到paths前（以id的形式）
     * @param start_id 前置lanelet id
     * @param paths 待插入的所有路径
     * @return 插入完成的所有路径
     */
   std::vector<lanelet::Ids> LinkLanes(const lanelet::Id start_id, std::vector<lanelet::Ids> paths);

   /**
     * @brief 在给定路径长度的情况下，找到所有不需要换道的后继路径（包括起始路径）
     * @param map [地图]
     * @param routing_map [路由图]
     * @param start_id [起始路径]
     * @param max_distance [规划路径的最大长度]
     */
   std::vector<lanelet::Ids> GetFollowsInDistance(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, lanelet::Id start_id, double max_distance);

   /**
     * @brief 在路由图中寻找，对应给定的lanelet id，是否有左右车道（可换道），和左右车道id
     * @param lane_id [本车道id]
     * @param map [地图]
     * @param besides [output][ 一个pair，first为左车道id，second为右车道id]
     * @return [一个pair，first为是否有左车道，second为是否有右车道]
     */
   std::pair<bool, bool> GetSlice(lanelet::Id lane_id, const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, std::pair<lanelet::Id, lanelet::Id> &besides);

   /**
   * @brief 在给定的path(以ids表示)中，找到最近的一个道路的索引
   * @param path 给定的path道路
   * @param point 给定的点
   * @param map 地图
   * @return int类型的索引
   */
   unsigned int GetClosestIndexInPath(const std::vector<int> path, const lanelet::Point3d point, const lanelet::LaneletMapUPtr &map);

   /**
     * @brief 对应给定的点和中心线，找到中心线上距离该点最近的点的索引，以及中心线剩余长度
     * @param point 给定的点
     * @param cline 给定的中心线
     * @return [一个pair，first为剩余长度，second为最近点索引]
     */
   std::pair<double, int> GetRemainDistance(const lanelet::ConstPoint3d point, const lanelet::ConstLineString3d cline);

   /**
     * @brief 将一系列以ids表达的路径，转换为以一条points表达一条路径的形式，每条路径根据给定的方向打上tag（左右转和直行）
     * @param map [地铁]
     * @param dir [给定方向]
     * @param paths [原始所有路径]
     * @return 一个vector，里面的每个pair是一条路径和路径方向构成的
     */
   std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> Ids2Lines(const lanelet::LaneletMapUPtr &map, lanelet2_ex::Trajectory_dir dir, std::vector<lanelet::Ids> paths);

   /**
     * @brief 将一系列以ids表达的路径，转换为以一条points表达一条路径的形式，每条路径根据给定的方向打上tag（左右转和直行,以及到中心线距离）
     * @param map [地铁]
     * @param dir [给定方向]
     * @param paths [原始所有路径]
     * @return 一个vector，里面的每个pair是一条路径和路径方向构成的
    */
   std::vector<std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr>> Ids2LinesWithTag(const lanelet::LaneletMapUPtr &map, lanelet2_ex::Lan_Atr attr, std::vector<lanelet::Ids> paths);

   /**
     * @brief 将一系列离散的lanelet ids中，存在前驱后继关系的链接起来组成一条路径
     * @param map [地图]
     * @param ids [离散车道ids]
     * @return 多条路径组成的vector
     */
   std::vector<lanelet::Ids> LinkFollows(const lanelet::LaneletMapUPtr &map, lanelet::Ids ids);

   /**
     * @brief constline转为一系列的constpoint
     * @param cline [input constline]
     * @return constpoints，系列点
     */
   std::vector<legionclaw::interface::Point3D> Cline2Points(lanelet::ConstLineString3d cline);
   /**
     * @brief 根据定位点找到所有可能的轨迹
     * @param map [地图]
     * @param routing_map[路由图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param bDirectionBased [是否考虑航向角]
     * @param max_distance [搜索轨迹的最大长度]
     * @return 轨迹的vector(一条轨迹是由一条中心线和中心线方向组成的pair)
     */
  //  std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> GetRoute(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, const double max_distance);

   /**
     * @brief 根据定位点找到所有可能的轨迹
     * @param map [地图]
     * @param routing_map[路由图]
     * @param point [坐标，虽然是gps点，但是目前传入ENU坐标，没有z坐标]
     * @param theta [航向，正北为0，顺时针增加]
     * @param max_distance [搜索轨迹的最大长度]
     * @param range [搜索范围]
     * @return 轨迹列的vector(一条轨迹是由一条中心线和中心线方向组成的pair)(一个轨迹列对应一个车身定位到的具体车道的路由)
     */
   std::vector<std::vector<std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr>>> GetRoutes(const lanelet::LaneletMapUPtr &map,
                                                                                               const lanelet::routing::RoutingGraphUPtr &routing_map,
                                                                                               const lanelet::Point3d point, const double theta, const bool use_theta,
                                                                                               const double max_distance, const double ele_diff_threshold, bool &in_lane);
} //namespace lanelet2_ex
#endif