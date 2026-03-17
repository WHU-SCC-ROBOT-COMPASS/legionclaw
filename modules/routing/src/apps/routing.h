/**
 * @file    routing.h
 * @author  zdhy
 * @date    2021-10-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <thread>
#include <iomanip>

#include "modules/common/json/json.hpp"
#include "modules/common/status/status.h"
#include "modules/common/logging/logging.h"
#include "message_manager/message_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/routing/src/common/local_view.h"
#include "modules/routing/src/common/routing_gflags.h"
#include "modules/common/base_message/message.pb.h"

#include <string>
#include <thread>
#include <fstream>
#include <iostream>
#include "modules/common/timer/timer_manager.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/time/time_tool.h"
#include "modules/routing/src/message_manager/message_manager.h"

#include "lanelet2_ex.h"
#include "modules/common/json/json.hpp"
#include "modules/common/status/status.h"
#include "modules/common/enum/enum.h"
#include "lanelet2_io/Io.h"
#include "projector.hpp"
#include "transformer.hpp"
#include "lanelet2_routing/Route.h"
#include "lanelet2_projection/UTM.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingCost.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_traffic_rules/TrafficRules.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/BoundingBox.h"
#include "lanelet2_routing/RoutingGraphContainer.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"
#include "boost/geometry.hpp"

#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/routing_request.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/stop_info.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/math/math_utils.h"
#include "modules/common/interface/guide_info.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/navi_info_msg.hpp"
#include "modules/common/fault/fault_client.hpp"

#if LCM_ENABLE
#include "message_manager/lcm/lcm_message_manager.h"
#endif
#if DDS_ENABLE
#include "message_manager/dds/dds_message_manager.h"
#endif
#if ROS_ENABLE
#include "message_manager/ros/ros_message_manager.h"
#endif
#if ROS2_ENABLE
#include "message_manager/ros2/ros2_message_manager.h"
#endif

// #include "conf/routing_conf.hpp"
#include "modules/routing/src/proto/routing_conf.pb.h"
/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */
#define RAD2DEG 180. / M_PI
namespace legionclaw
{
  namespace routing
  {
    using namespace legionclaw::common;
    using json = nlohmann::json;

    struct ParkStop
    {
      interface::KeyPoint stop_point_;      //@brief 泊车前停止点
      interface::ParkingInfo parking_info_; //@brief 停车点信息
    };

    struct StopData//@brief 一条停车线的相关信息
    {
      int line_id;//@brief 停止线id
      std::vector<int>  lane_ids; //@brief 需要检测的lanelet的id;
      std::vector<legionclaw::interface::Point3D> stop_points;//@brief 每条车道对应的停车点;
      legionclaw::common::Direction direction;//@brief 停止线方向
      legionclaw::common::TrafficLightColor color;//@brief 对应红绿灯颜色
    };

   enum LaneInfoType
   {
      LANE_TYPE_UNKNOWN = 0,
      LEFT_TURN_NO_TURN_AROUND = 1,
      STRAIGHT = 2,
      RIGHT_TURN = 3,
      STRAIGHT_AND_LEFT_TURN = 4,
      STRAIGHT_AND_RIGHT_TURN = 5,
      LEFT_TURN_AND_TURN_AROUND = 6,
      NO_LEFT_TURN_ONLY_TURN_AROUND = 7
   };

    /**
 * @class Routing
 * @brief 控制类.
 */
    class Routing
    {
    public:
      Routing(std::string file_path) : config_file_path_(file_path){};
      ~Routing() = default;
      /**
   * @brief     初始化．
   * @param[in] void．
   * @return    void.
   */
      void Init();

      /**
   * @brief     join．
   * @param[in] void.
   * @return    void.
   */
      void Join();

      /**
   * @brief Get the Conf object
   * @return std::shared_ptr<RoutingConf>
   */
      std::shared_ptr<RoutingConf> GetConf() const;

    protected:
      //初始化状态
      bool is_init_;
      // 功能激活状态,激活为true，未激活为false
      bool function_activation_;

      std::mutex mutex_;
      //配置文件路径
      std::string config_file_path_;
      //配置文件操作类
      json routing_json_;
      //控制逻辑设置
      std::shared_ptr<RoutingConf> routing_conf_;
      //消息控制器
      std::map<std::string, std::shared_ptr<MessageManager<Routing>>>
          message_manager_;
      //thread
      std::unique_ptr<std::thread> task_thread_;
      //navi key point
      bool new_goal_received_;           //@brief 是否收到新的目标点
      bool start_pose_inited_;           //@brief 起始点是否初始化
      interface::Location current_pose_; //@brief 车身当前定位和朝向
      interface::KeyPoint goal_pose_;    //@brief 收到的目标点：lon表示x，lat表示y
      interface::Chassis vehicle_state_; //@brief 车辆当前状态（油门值和车速m/s）
      interface::TrafficLightMsg traffic_light_msg_; //@brief 当前红绿灯消息（感知的结果）
      legionclaw::local_map::transform::Transformer transformer_; //@brief 接收到的tf消息
      bool transformer_init_=false; //@brief 是否接收到tf消息
      //navi output stopinfo and parkinginfo
      interface::StopInfo stop_info_;       //@brief 停车点信息
      std::vector<int> stop_lane_ids_; //@brief 停车点匹配到的道路id，用于车辆到站判断
      bool match_stop_points_; //@brief 停止点和中心线匹配是否成功
      bool stop_check_;//@brief 开始停止点检测的标志
      std::vector<interface::LaneInfo> stop_lanes_;//停车点所在中心线
      // std::vector<interface::LanePoint> stop_points_;//停车点所匹配到的点
      std::vector<int> stop_indexs_;//停车匹配点的索引
      interface::ParkingInfo parking_info_; //@brief 车位信息
      bool location_init_;

      //new  interface msg
      bool new_request_received_; //@brief 是否接受到新的规划请求
      bool replan_flag_;          //@brief 重规划标志
      bool new_parking_space_;    //@biref选择到了新的停车位

      interface::RoutingResponse response_; //@brief 下发的切片规划结果

      interface::Location location_;
      interface::Chassis chassis_;
      interface::RoutingRequest routing_request_; //@brief 接收到到的规划请求
      interface::JunctionInfo cur_junc_;
      interface::TrafficEvents cur_event_;
      interface::LimitSpeedInfo cur_speed_limit;

      //parking info
      std::vector<ParkStop> parking_areas_;

      //map
      bool mileage_enable_;                                  //@brief 开始里程计的标志
      bool map_loaded_;                                      //@brief 地图是否加载成功
      bool first_send_;                                      //第一次下发标志
      myprojector::Projector projector_;                     //@brief utm投影的转换
      lanelet::LaneletMapUPtr map_;                          //@brief 地图(lanelet2格式osm地图)
      lanelet::routing::RoutingGraphUPtr routing_map_;       //@brief 路由图
      lanelet::LaneletMapConstPtr const_map_;         //@brief const地图
      lanelet::traffic_rules::TrafficRulesPtr traffic_rule_; //@brief 构建路由图的交通规则

      //用来做障碍物过滤的局部地图
      std::vector<int> count_id;            //@brief 使用过的车道id   
      std::vector<int> merged_id;        //@brief 合并过的车道id
      lanelet::LaneletMapUPtr localmap;            //@brief 局部地图
      lanelet::routing::RoutingGraphUPtr local_routing;         //@brief 局部路由图

      //全局路由消息生成
      // bool send_global_; //@brief 下发全局路由消息的标志
      interface::GlobalRouteMsg global_route_msg_; //@brief 全局路由消息
      interface::LaneletInfo global_lane_info_; //@brief 一次导航的道路信息
      interface::LaneletInfo current_laneletinfo_; //@ brief 当前所在车道的信息
      interface::LaneletInfo besides_laneletinfo_; //@ brief 邻近车道信息
      double total_mileage_; //@brief 总里程
      double cur_mileage_; //@brief 当前里程
      legionclaw::interface::NaviInfoMsg navi_info_msg_;

      //routing
      std::map<int, int> path_id_map_;          //@brief 对车道进行pid分类后的图，由lanelet id 对应到path id
      std::vector<std::vector<int>> local_ids; //@brief 全局规划的切片结果，车道级SS

      //下发切片地图
      bool send_flag_; //@brief 是否下发切片
      bool junction_check_;   //@brief 是否开始检测路口红绿灯停止线
      // interface::Location section_start_point_;	//@brief 下发一次切片后或者进行一次规划后将当前点作为里程记的起点
      interface::Location section_last_point_;    //@brief 存储上一个定位点，做里程计使用
      double section_mileage_;                    //@brief 局部里程计
      std::vector<std::vector<int>> local_paths_; //@brief 下发的局部切片结果
      // std::map<std::string, legionclaw::common::MessageStatus> message_status_;//@brief  消息状态
      std::vector<StopData> all_stop_lines_;//@brief 一次规划中的所有停止线信息
      std::vector<std::vector<int>> one_way_plan_;    //@brief 一次规划中的所有路口转向信息,最后一段路是转向或者直行车道
      std::vector<int> no_change_lane_;//剔除最短路径下的左右换道路段
      lanelet::Optional<lanelet::routing::Route> route;//路由图
      std::vector<legionclaw::common::Direction> junctions_directions_;      //@brief 和路口数量对应的路口转向方向

      legionclaw::interface::Faults faults_;
      legionclaw::interface::FaultCodeSet *faultcodeset_;
      // 定时器
      std::shared_ptr<ADTimerManager<Routing, void>> ad_timer_manager_;
      std::shared_ptr<WheelTimer<Routing, void>> task_mainloop_;
      std::shared_ptr<WheelTimer<Routing, void>> task_check_station_;
      std::shared_ptr<WheelTimer<Routing, void>> task_check_stopline_1000ms_;
      std::shared_ptr<WheelTimer<Routing, void>> task_merge_polygons_1000ms_;
      std::shared_ptr<WheelTimer<Routing, void>> task_global_msg_gen_1000ms_;
      std::shared_ptr<WheelTimer<Routing, void>> task_guide_info_gen_1000ms_;
      std::shared_ptr<WheelTimer<Routing, void>> task_lanelines_gen_1000ms_;
      std::shared_ptr<WheelTimer<Routing, void>> task_navi_info_gen_100ms_;

#if LCM_ENABLE
      std::shared_ptr<LcmMessageManager<Routing>> lcm_message_manager_;
#endif
#if DDS_ENABLE
      // DDS消息控制器
      std::shared_ptr<DdsMessageManager<Routing>> dds_message_manager_;
#endif
#if ROS_ENABLE
      std::shared_ptr<RosMessageManager<Routing>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<Routing>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
      std::shared_ptr<AdsfiMessageManager<Routing>> adsfi_message_manager_;
#endif

    protected:
      /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
      void ResigerMessageManager(
          std::string name,
          std::shared_ptr<MessageManager<Routing>> message_manager);

      /**
   * @brief     消息初始化.
   * @return    void.
   */
      void MessagesInit();


      /**
       * @brief     消息激活.
       * @return    void.
       */
      void MessagesActivate();

      /**
       * @brief     消息去激活.
       * @return    void.
       */
      void MessagesDeActivate();

      /**
       * @brief 定时器任务激活
       */
      void TaskActivate();

      /**
       * @brief 定时器任务停止
       */
      void TaskStop();

      /**
       * @brief 故障码监控初始化
       */
      void FaultMonitorInit();


      /**
   * @brief
   * @param  routing_response
   */
      void
      PublishRoutingResponse(legionclaw::interface::RoutingResponse routing_response);

      /**
   * @brief
   * @param  parking_info
   */
      void PublishParkingInfo(legionclaw::interface::ParkingInfo parking_info);

      /**
   * @brief
   * @param  stop_info
   */
      void PublishStopInfo(legionclaw::interface::StopInfo stop_info);

      /**
      * @brief
      * @param  guide_info
      */
      void PublishGuideInfo(legionclaw::interface::GuideInfo guide_info);

      /**
      * @brief
      * @param  navi_info_msg
      */
      void PublishNaviInfoMsg(legionclaw::interface::NaviInfoMsg navi_info_msg);

      /**
      * @brief
      * @param  lane_list
      */
      void PublishLaneList(legionclaw::interface::LaneList lane_list);
	  
	  /**
	   * @brief
	   * @param  traffic_light_msg_output
	   */
  	  void PublishTrafficLightMsgOutput(legionclaw::interface::TrafficLightMsg traffic_light_msg_output);

      /**
   * @brief
   * @param  faults
   */
      void PublishFaults();

      /**
       * @brief
       * @param  global_route_msg
       */
      void PublishGlobalRouteMsg(legionclaw::interface::GlobalRouteMsg global_route_msg);

      /**
       * @brief
       * @param  polygon_2d
       */
      void PublishPolygon2D(legionclaw::interface::Polygon2D polygon_2d);

      /**
       * @brief
       * @param  traffic_events
       */
      void PublishTrafficEventsOutput(legionclaw::interface::TrafficEvents traffic_events_output);

   public:
      /**
       * @brief     TrafficLightMsg消息接收.
       * @param[in] traffic_light_msg .
       * @return    void.
       */
      void HandleTrafficLightMsg(legionclaw::interface::TrafficLightMsg traffic_light_msg);
      /**
       * @brief     Odometry消息接收.
       * @param[in] ins .
       * @return    void.
       */
      void Routing::HandleOdometry(legionclaw::interface::Odometry odometry);
      /**
   * @brief     Location消息接收.
   * @param[in] location .
   * @return    void.
   */
      void HandleLocation(legionclaw::interface::Location location);

      /**
   * @brief     Chassis消息接收.
   * @param[in] chassis .
   * @return    void.
   */
      void HandleChassis(legionclaw::interface::Chassis chassis);

      /**
   * @brief     ObuCmdMsg消息接收.
   * @param[in] obu_cmd_msg .
   * @return    void.
   */
      void HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg);


   /**
   * @brief     TrafficEvents消息接收.
   * @param[in] traffic_events .
   * @return    void.
   */
  void HandleTrafficEventsInput(legionclaw::interface::TrafficEvents traffic_events_input);
      /**
   * @brief     RoutingRequest消息接收.
   * @param[in] routing_request .
   * @return    void.
   */
      void HandleRoutingRequest(legionclaw::interface::RoutingRequest routing_request);


      /**
   * @brief    红绿灯消息接收.
   * @param[in] traffic lights .
   * @return    void.
   */
      void HandleTrafficLights();

    protected:
      /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
      void Spin();

      /**
			 * @brief 加载lanelet2地图的函数
			 */
      void LoadLanelet2Map();

      /**
			 * @brief 加载所有停止线和红绿灯信息
			 */
      void LoadStopLines();

       /**
          * @brief 根据车道id判断是否使用过
          * @param lane_id [in][输入的lanelet车道id]
          */
      bool UsagedIndex(int lane_id);

      /**
          * @brief 根据车道id判断是否合并过
          * @param lane_id [in][输入的lanelet车道id]
          */
      bool MergedIndex(int lane_id);

      /**
          * @brief 将车道合并到已有的车道polygon中
          * @param existing_polygon [in][输入的polygon]
          * @param lanelet [in][输入的lanelet车道]
          * @return res_polygon 
          */
      lanelet::BasicPolygon2d merger(lanelet::BasicPolygon2d existing_polygon, lanelet::Lanelet lanelet);

      /**
          * @brief 合并周围车道
          * @param existing_polygon [in][输入当前已有的车道polygon]
          * @param lanelet [in][输入起始的lanelet车道]
          * @return res_polygon 
          */
      lanelet::BasicPolygon2d MergeLoop(lanelet::BasicPolygon2d res_polygon, lanelet::Lanelet start_lane);

      /**
          * @brief 合并范围内的车道polygon
          */
      void MergePolygons(void *param);

      /**
          * @brief 全局路由消息生成
          */
      void GlobalMsgGen(void *param);

      /**
			 * @brief 判断停止线信息，快到路口，以及下发停止线红绿灯
			 */
      void CheckStopLine(void *param);

      /**
       * @brief 输出全局导航规划下当前道路引导信息
       */
      void GuideInfoGen(void *param);

      /**
       * @brief 输出全局导航规划下当前定位附近的车道边线信息
       */
      void LanelinesGen(void *param);

      /**
       * @brief 输出全局导航规划下当前定位附近的车道信息
       * */
      void NaviInfoPublisher(void *param);

      //@brief 找出id对应的下一段路的转弯方向，用于停止线对应的转向判别
      legionclaw::common::Direction FindNextJunctionDirection(int id);


      //@brief 找出id对应的本路的转弯方向，用于转向提醒
      legionclaw::common::Direction FindJunctionDirection(int id);

      //@brief 找出id对应的车道的转向引导信息，用于LaneInfoType的赋值
      int GetLaneInfoType(int id);
      //@brief 找出id对应的车道的道路信息，用于RoadType的赋值
      int GetLaneRoadType(int id);

      //@brief 加载所有路口转向信息,接收一次shortest path
      void LoadJunctionInfo(const std::vector<int> &ids);

      /**
			 * @brief 全局规划函数
			 * @param request 规划请求信息（关键点列表）
			 * @param paths [output 输出的规划路径]
			 * @return PlanStatus值，是否规划成功
			 */
      legionclaw::interface::RoutingResponse::PlanStatus GlobalPlan(interface::RoutingRequest request);

      /**
       * @brief 把一条最短路径，扩充为左右可走的全路由
       * @param single_road 最短路径
       * @param graph 路由图
       * @param local_route 局部全路由
       */
      bool SingleRoad2FullRoute(const std::vector<int> &single_road, std::vector<std::vector<int>> &local_route);

      /**
			 * @brief 根据定位点将全部规划进行切片
			 * @param singleroad [in][单条路径，以lanelet_id的list表示]
			 * @param local_route [out][输出的切片路径]
			 * @param pose[in][当前位置]
			 * @param back_length[in][车身后方下发的路由长度]
			 * @param forward_length [in][车身前方下发的路由长度]
			 */
      legionclaw::interface::RoutingResponse::PlanStatus GetSection(const std::vector<int> &singleroad, std::vector<std::vector<int>> &local_route, const interface::Location pose, const double back_length, const double forward_length);

      /**
			 * @brief 局部规划的结果转化成最终下发的response
			 * @param local_route [in][输入的局部规划结果]
			 * @param response [out][输出的response消息]
			 */
      void Section2Response(const std::vector<std::vector<int>> &local_route, interface::RoutingResponse &response);

      /**
			 * @brief 计算车辆是否到达停车点
			 * @param goal 目标停车点位置//keypoint：lon表示x，lat表示y
			 * @param pose 车辆当前定位
			 * @param chassis 底盘信息
			 */
      void InStation(const interface::KeyPoint goal, const interface::Location pose, const interface::Chassis chassis);

      /**
			 * @brief 将经纬度表示的KeyPoint 转化为xyz表示的keypoint：lon表示x，lat表示y
			 * @param keypoint 输入
			 * @return new keypoint
			 */
      interface::KeyPoint ll2xy(const interface::KeyPoint keypoint);

      /**
			 * @brief 生成停车点信息
			 * @param goal 以xy表示的keypoint点，lon表示x，lat表示y
			 * @return stopinfo
			 */
      interface::StopInfo GenStopInfo(const interface::KeyPoint goal);

      /**
			 * @brief 状态与缓存清理函数
			 */
      void Clear();

      /**
			 * @brief 主循环，监测是否有新的规划请求，有则进行一次新的规划
			 */
      void MainLoop(void *param);

      /**
			 * @brief 获取lane在集合中的索引
			 * @param lanes,lane
          * @return index
			 */
      int GetIndex(lanelet::ConstLanelets lanes, lanelet::ConstLanelet &lane);

      /**
			 * @brief 根据x,y,z,yaw获取lane
			 * @param x,y,yaw
			 */
      bool GetLane(double x, double y, double z, double yaw, lanelet::ConstLanelet &lane);

      /**
			 * @brief 泊车前规划,选择最近的泊车位进行规划
			 */
      bool ParkingPlan();

      /**
			 * @brief 更新地图所有停车位信息
			 */
      void LoadParkingAreas();
      /**
       * @定时检测是否下发停车点
       */
      void Check_Sation_Loop(void *param);

      /**
       * @brief 给定x，y，z，yaw，匹配到中心线上最近的点索引,遍历消耗较大，应该只用于终点匹配,返回-1代表查找失败
       */
      int GetNearestPointInPath(double x,double y,double z,double yaw,const interface::LaneInfo &lane);

      /**
       * @brief 给定x，y，z，yaw,range以及终点索引index，去判断是否快到终点
       */
      bool MatchPointInPath(double x,double y,double z,double yaw,const double range, const double height_range,const double max_distance,const double index,const interface::LaneInfo &lane);


      void CalculateStopPoint();

      void FindSuitableStopLane(lanelet::Lanelet lane, lanelet::Lanelets &lanes, const double range);

      int MatchTrafficlight(legionclaw::common::Direction dir,const std::vector<legionclaw::interface::TrafficLight>& lights);

      // 根据中心点、长宽和旋转角度，计算矩形顶点
      std::vector<legionclaw::interface::Point2D> getRectVertex(legionclaw::interface::Point2D center,float theta,float w,float h);
      // 判断点是否在多边形内
      bool PointInPolygon(legionclaw::interface::Point2D p, std::vector<legionclaw::interface::Point2D>& ptPolygon, int nCount);

    };

      /**
		 * @brief 依靠从左到右的顺序对各个车道进行path id的赋值，最内侧为1,往外（右）递增，同时存在前后车道的数值的继承，此函数为递归函数
		 * @param start_id [起始的lanelet id]
		 * @param all_ids [所有可能的车道 id集合]
		 * @param lanes [id的集合构成一条线路（path id相同），多个ids构成可行驶的路由]
		 * @param map [output 车道的lanelet id与path id对应的map]
		 * @param graph [起点到终点的路由图]
		 * @param osm_map [lanelet2 地图]
		 * @return bool值，path id 赋值是否成功
		 */
      void GetPathID(int start_id, std::vector<int> all_ids, std::vector<std::vector<int>> &lanes, std::map<int, int> &map, lanelet::routing::RoutingGraphUPtr &graph, lanelet::LaneletMapUPtr &osm_map);

   //  /**
	// 	 * @brief 依靠从左到右的顺序对各个车道进行path id的赋值，最内侧为1,往外（右）递增，同时存在前后车道的数值的继承，此函数为递归函数
	// 	 * @param start_id [起始的lanelet id]
	// 	 * @param all_ids [所有可能的车道 id集合]
	// 	 * @param lanes [id的集合构成一条线路（path id相同），多个ids构成可行驶的路由]
	// 	 * @param map [output 车道的lanelet id与path id对应的map]
	// 	 * @param graph [起点到终点的路由图]
	// 	 * @param osm_map [lanelet2 地图]
	// 	 * @return bool值，path id 赋值是否成功
	// 	 */
   //  bool GetPathID(int start_id, std::vector<int> all_ids, std::vector<std::vector<int>> &lanes, std::map<int, int> &map, lanelet::routing::RoutingGraphUPtr &graph, lanelet::LaneletMapUPtr &osm_map);

    /**
		 * @brief 对引导线进行平滑处理（通过前后各一个点来修正当前点）
		 * @param path 引导线（点的vector）[input & output]
		 * @param weight_data 原始当前点数据所占权重
		 * @param weight_smooth 前后点数据所占权重
		 * @param tolerance 容忍值，当修正前后的位置偏差仍然大于容忍值时会继续进行修正
		 */
    void SmoothPath(interface::LaneInfo &lane_info, double weight_data, double weight_smooth, double tolerance);

    /**
		 * @brief 对引导线进行插值处理
		 * @param path 引导线（点的vector）[input & output]
		 * @param distanceDensity 密度[单位为m] 
		 */
    void FixPathDensity(interface::LaneInfo &lane_info, const double &distanceDensity);

    /**
		 * @brief 找到lists中的包含input的list的索引
		 * @param input 要寻找的int值
		 * @param lists int组成的vector为list list组成的vector为lists
		 */
    unsigned int GetIndex(int input, std::vector<std::vector<int>> lists);

    /**
		 * @brief 朝北为0，顺时针增加的0~360度的仰角（角度制），转化为：朝东为0，逆时针增加的-PI到PI的航向角（弧度制）
		 * @param input 输入的仰角
		 * @return 输出弧度制的新仰角
		 */
    double Yaw2Angle(double input);

    namespace unique
    {
      /**
		 * @brief 生成一个独一无二的seq id，递增，32位无符号整数，对应错误码消息
		 */
      uint32_t getFaultCodeSeq();
      /**
		 * @brief 生成一个独一无二的seq id，递增，32位无符号整数，对应停车点消息
		 */
      uint32_t getStopInfoSeq();
      /**
		 * @brief 生成一个独一无二的seq id，递增，32位无符号整数，对应泊车点消息
		 */
      uint32_t getParkingInfoSeq();
      /**
		 * @brief 生成一个独一无二的seq id，递增，32位无符号整数，对应路由结果回复消息
		 */
      uint32_t getRoutingResponseSeq();

    } //namespace unique
  }   // namespace routing
} // namespace legionclaw
