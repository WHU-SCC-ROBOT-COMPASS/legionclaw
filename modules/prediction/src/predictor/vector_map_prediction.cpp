#include "vector_map_predictor.h"
// #include "modules/prediction/src/common/enum_list.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time_tool.h"
// #include "modules/common/time/time_tool.h"
// #include "op_planner/MappingHelpers.h"
// #include "op_planner/PlanningHelpers.h"
// #include "op_planner/PlannerH.h"
//#include <math.h>
#include "modules/prediction/src/common/map_matcher/map_matcher.h"
#include "modules/prediction/src/common/spline/spline.h"
#include "modules/common/math/curve_math.h"

using namespace legionclaw::common;

/**
 * @brief Predict for pedestrian with osm
 */
bool VectorMapPrediction::PredictPedestrian(const lanelet::LaneletMapUPtr &map,
                                            const lanelet::routing::RoutingGraphUPtr &routing_map,
                                            const std::deque<legionclaw::interface::Obstacle> &object_deque,
                                            const PredictionConf &prediction_conf,
                                            const int &filter_mode, 
                                            std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    // double predict_time = prediction_conf.prediction_horizon() * prediction_conf.predict_time_interval();
    double predict_time = 1.0;  //对于行人、自行车等，预测时间由4s改为2s
    double predict_time_interval =  prediction_conf.predict_time_interval();
    // double filter_range =  prediction_conf.filter_range();
    int theta_mode = prediction_conf.theta_mode();
    // double ele_diff_threshold = prediction_conf.ele_diff_threshold();

    trajectories.clear();
    // std::vector<std::vector<PlannerHNS::WayPoint>> paths;
    legionclaw::interface::Obstacle object;
    if (object_deque.size() > 0)
        object = object_deque.front();

    double obj_v = hypot(object.velocity_abs().x(), object.velocity_abs().y());
    double obj_a = hypot(object.acceleration_abs().x(), object.acceleration_abs().y());
    if (isnan(obj_v) || isinf(obj_v))
        obj_v = 0.0;
    if (isnan(obj_a) || isinf(obj_a))
        obj_a = 0.0;
    // double prediction_distance = obj_v * predict_time; // + 0.5 * obj_a * predict_time * predict_time; //s=v*t+0.5*a*t^2
    double theta_start = object.theta_abs();
    //静止障碍物没必要通过速度计算方向角
    if (obj_v >= prediction_conf.threshold_static_speed() && (theta_mode == USE_VELOCITY_THETA || theta_mode == USE_MAP_THETA))
       theta_start = math::GetHeadingRadian(object.velocity_abs().x(), object.velocity_abs().y());

    // PlannerHNS::PlannerH planner;
    legionclaw::interface::TrajectoryPoint fake_pose;
    legionclaw::interface::PathPoint obstacle_point;
    obstacle_point.set_x(object.center_pos_abs().x());
    obstacle_point.set_y(object.center_pos_abs().y());
    obstacle_point.set_theta(theta_start);
    obstacle_point.set_kappa(0.0);
    obstacle_point.set_s(0.0);
    fake_pose.set_path_point(obstacle_point);
    fake_pose.set_v(obj_v);
    fake_pose.set_a(obj_a);

    // 目前将自行车行人等障碍物预测轨迹改为直线预测
    if (obj_v >= prediction_conf.threshold_static_speed())
    {
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }
    return true;

    /*if (filter_mode == NO_MAP)
    {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }

    // std::vector<double> ClosestDistance;
    // std::vector<std::pair<double, PlannerHNS::Lane *>> lanelist_bak; //暂时没有用到

    // std::vector<PlannerHNS::WayPoint *> closest_points = PlannerHNS::MappingHelpers::GetClosestWaypointsListFromMapWithGap(fake_pose, map, ClosestDistance,
    //                                                                                                                        lanelist_bak, 0, true);
    //地图匹配成功
    // int64_t t1 = TimeTool::Now2Us();  //for test
    lanelet::Point3d fake_point(lanelet::utils::getId(), object.center_pos_abs().x(), object.center_pos_abs().y(), object.center_pos_abs().z());
    // std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> refer_lines; //输出的中心线，待拟合,输入的地图信息来源参考map_convert/node/maintest
    // refer_lines = lanelet2_ex::GetRoute(map, routing_map, fake_point, v_theta, true, prediction_distance); //输出的中心线，待拟合
    std::vector<std::vector<std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr>>> refer_line_arry;
    bool in_lane = false;
    if(filter_mode == FILTER_BY_CENTER_POINT || filter_mode == FILTER_BY_POLYGON)
    {        
        // std::cout << "object.polygon_point_abs() : " << object.polygon_point_abs().size() << "\n";
        if(filter_mode == FILTER_BY_CENTER_POINT)
        {
            lanelet::Ids lane_ids = lanelet2_ex::GetClosestLaneletsInRange(map, fake_point, filter_range);
            if (lane_ids.size() > 0)
            {
                in_lane = true;
            }

        }
        else if(filter_mode == FILTER_BY_POLYGON)
        {
            for (auto polygon_point : object.polygon_point_abs())
            {
                lanelet::Point3d point(lanelet::utils::getId(), polygon_point.x(), polygon_point.y(), 0);
                lanelet::Ids lane_ids = lanelet2_ex::GetClosestLanelets(map, point);
                if (lane_ids.size() > 0)
                {
                    in_lane = true;
                    break;
                }
            }
        }        
        
        if (!in_lane)
        {
            return false;
        }
    }

    //静止障碍物不用再搜索路由
    if (obj_v < prediction_conf.threshold_static_speed())
        return true;

    refer_line_arry = lanelet2_ex::GetRoutes(map, routing_map, fake_point, 90.0 - math::R2D(theta_start), true,
                                             prediction_distance, ele_diff_threshold, in_lane); //write range into congig
    // int64_t t3 = TimeTool::Now2Us();  //for test

    if (refer_line_arry.size() > 0)
    {
        // if (obj_v < prediction_conf.threshold_static_speed())
        //     return true;

        legionclaw::prediction::PredictionTrajectory trajectory;
        legionclaw::interface::TrajectoryInPrediction predict_trajectory;
        legionclaw::interface::PathPoint start_point, end_point;
        double min_dis = DBL_MAX;
        int min_i = 0, min_j = 0;
        for (unsigned int i = 0; i < refer_line_arry.size(); i++)
        {
            for (unsigned int j = 0; j < refer_line_arry[i].size(); j++)
            {
                if (refer_line_arry[i][j].first.size() && fabs(refer_line_arry[i][j].second.dis) < min_dis)
                {
                    min_dis = fabs(refer_line_arry[i][j].second.dis);
                    min_i = i;
                    min_j = j;
                }
            }
        }
        //spline
        auto raw_line = refer_line_arry[min_i][min_j];
        std::vector<legionclaw::interface::Point3D> line;
        double dis = 0.0;
        line.clear();
        legionclaw::interface::Point3D raw_point;
        for (unsigned int k = raw_line.second.index; k < raw_line.first.size(); k++)
        {
            raw_point = raw_line.first.at(k);
            if (!line.empty())
                dis += math::Norm(raw_point.x() - line.back().x(), raw_point.y() - line.back().y());
            if (dis > prediction_distance && line.size() >= 5)
                break;
            line.push_back(raw_point);
        }
        int line_num = line.size();
        double heading_start, heading_end;
        if (line_num < 2)
        {
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
            return true;
        }
        heading_start = math::GetHeading(line.at(0).x(), line.at(0).y(),
                                            line.at(1).x(), line.at(1).y());
        heading_end = math::GetHeading(line.at(line_num - 2).x(), line.at(line_num - 2).y(),
                                        line.back().x(), line.back().y());

        start_point.set_x(line.front().x());
        start_point.set_y(line.front().y());
        start_point.set_theta(math::NormalizeAngle(math::D2R(90.0 - heading_start)));
        start_point.set_kappa(0.0);

        end_point.set_x(line.back().x());
        end_point.set_y(line.back().y());
        end_point.set_theta(math::NormalizeAngle(math::D2R(90.0 - heading_end)));
        end_point.set_kappa(0.0);

        // int64_t t4 = TimeTool::Now2Us();  //for test
        std::vector<legionclaw::interface::PathPoint> ref_path, predict_path;
        spiral::CubicSpiralCurve csc(start_point, end_point);
        csc.SetSpiralConfig(spiral_params_);
        csc.CalculatePath();
        double dx = end_point.x() - start_point.x();
        double dy = end_point.y() - start_point.y();
        int num = (int)(predict_time / predict_time_interval);
        csc.GetPathVec(num, &ref_path);
        // int64_t t5 = TimeTool::Now2Us();  //for test

        if (ref_path.empty())
            return false;

        //match
        double dist_min;
        int matched_index = MapMatcher::QueryVerticalDistanceWithBuffer(ref_path, {obstacle_point.x(), obstacle_point.y()},
                                                            obstacle_point.theta(), 1.0e-6, dist_min);
        // int64_t t6 = TimeTool::Now2Us();  //for test

        //translate
        TranslateLine(ref_path, matched_index, dist_min, predict_path);
        // int64_t t7 = TimeTool::Now2Us();  //for test

        //path 2 trajectory
        legionclaw::interface::TrajectoryPoint trajectory_point;
        predict_trajectory.clear_trajectory_points();
        for (unsigned int i = 0; i < predict_path.size(); i++)
        {
            trajectory_point.set_path_point(predict_path.at(i));
            
            trajectory_point.set_relative_time(predict_time_interval * i);
            trajectory_point.set_v(obj_v);
            trajectory_point.set_a(obj_a);
            
            predict_trajectory.add_trajectory_points(trajectory_point);
            predict_trajectory.set_probability(1.0);
        }
        trajectory.set_trajectory_in_prediction(predict_trajectory);
        trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VECTORMAP);
        trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_STRAIGHT);
        trajectories.push_back(trajectory);
    }
    else
    {
        // if (obj_v < prediction_conf.threshold_static_speed())
        //     return true;
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }
    // int64_t t4 = TimeTool::Now2Us();  //for test
    // std::cout << "t12 = " << t2-t1 << "\n";
    // std::cout << "t23 = " << t3-t2 << "\n";
    // std::cout << "t34 = " << t4-t3 << "\n";
    return true;*/
}

bool VectorMapPrediction::GeneratePath(const std::vector<legionclaw::interface::Point3D> &center_line,
                const std::vector<double> &mileage_list,
                const legionclaw::interface::PathPoint &start_point,
                const double &delta_mileage,
                std::vector<legionclaw::interface::PathPoint> &predict_path)
{
    if (mileage_list.size() <= 2 || (center_line.size() != mileage_list.size()))
        return false;

    legionclaw::prediction::spline::spline spline_s_x, spline_s_y;

    //****重采样
    std::vector<double> x, y, s;
    x.clear();
    y.clear();
    s.clear();
    double mileage_threshold = 0.5 * mileage_list.back();                        
    double last_mileage = mileage_list[0];
    double last_theta = math::GetHeading(center_line[0].x(), center_line[0].y(),
                                            center_line[1].x(), center_line[1].y());
    double theta = 0.0;
    //第一个点取障碍物当前状态
    s.push_back(last_mileage);
    x.push_back(start_point.x());
    y.push_back(start_point.y());
    for(size_t i = 1; i < center_line.size(); i++)
    {
        if (mileage_list[i] - mileage_list[i - 1] <= 0)
            continue;
        
        theta = math::GetHeading(center_line[i - 1].x(), center_line[i - 1].y(),
                                    center_line[i].x(), center_line[i].y());
        if (mileage_list[i] - last_mileage > mileage_threshold
        || fabs(theta - last_theta) > 20.0)
        {
            last_mileage = mileage_list[i];
            last_theta = theta;

            s.push_back(mileage_list[i]);
            x.push_back(center_line[i].x());
            y.push_back(center_line[i].y());
        }
    }
    if (fabs(s.back() - mileage_list.back()) > 0.1*mileage_list.back())
    {
        s.push_back(mileage_list.back());
        x.push_back(center_line.back().x());
        y.push_back(center_line.back().y());
    }

    //******spline拟合
    if (s.size() <= 2) 
        return false;
    spline_s_x.set_points(s, x);
    spline_s_y.set_points(s, y);
    predict_path.clear(); 

    double heading = 0;
    double ks = DBL_MIN;
    double x_s, y_s;
    double first_deriv_x, second_deriv_x, first_deriv_y, second_deriv_y;
    legionclaw::interface::PathPoint smooth_point;
    for (double cs = s[0];cs <= s.back();cs += delta_mileage)
    {
        spline_s_x.compute_spline(cs, x_s, first_deriv_x,
            second_deriv_x);  // x(s)一阶段,二阶段
        spline_s_y.compute_spline(cs, y_s, first_deriv_y,
                                    second_deriv_y);  // y(s)一阶段,二阶段
        heading = math::GetHeadingRadian(first_deriv_x, first_deriv_y);
        // ks = compute_ks_from_spline(first_deriv_x, second_deriv_x, first_deriv_y,
        // second_deriv_y);
        ks = math::CurveMath::ComputeCurvature(first_deriv_x, second_deriv_x,
                                                first_deriv_y, second_deriv_y);
        smooth_point.set_s(cs);
        smooth_point.set_theta(math::NormalizeAngle(heading));
        smooth_point.set_kappa(ks);
        smooth_point.set_x(x_s);
        smooth_point.set_y(y_s);
        smooth_point.set_z(0.0);

        predict_path.push_back(smooth_point);
    }                                          

    return true;
}

bool VectorMapPrediction::GeneratePath(const std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr> &refer_line,
                const std::vector<double> &mileage_list,
                const legionclaw::interface::PathPoint &start_point,
                const double &delta_mileage,
                std::vector<legionclaw::interface::PathPoint> &predict_path)
{
    std::vector<legionclaw::interface::Point3D> center_line = refer_line.first;
    if (mileage_list.size() <= 2 || (center_line.size() != mileage_list.size()))
        return false;

    legionclaw::prediction::spline::spline spline_s_x, spline_s_y;

    //****重采样
    int num = center_line.size();
    std::vector<double> x, y, s;
    x.clear();
    y.clear();
    s.clear();
    double mileage_threshold = 5.0;//0.25 * mileage_list.back();                        
    double last_mileage = mileage_list[0];
    // double last_theta = math::GetHeadingRadian(center_line[1].x() - center_line[0].x(), 
    //                                            center_line[1].y() - center_line[0].y());
    double theta = 0.0;
    double lat_offset = 0.0;
    // double coef = refer_line.second.theta_diff / 90.0;  //和航向相关
    double coef = 0.0;
    // std::cout << "------------ " << refer_line.second.dis / mileage_list.back() << "\n";
    double total_s = std::max(mileage_list.back(), fabs(refer_line.second.dis) * 10);
    //第一个点取障碍物当前状态
    s.push_back(last_mileage);
    x.push_back(start_point.x());
    y.push_back(start_point.y());
    for(size_t i = 1; i < center_line.size(); i++)
    {
        if (mileage_list[i] - mileage_list[i - 1] <= 0)
            continue;
        
        theta = math::GetHeadingRadian(center_line[i].x() - center_line[i - 1].x(), 
                                       center_line[i].y() - center_line[i - 1].y());
        if (mileage_list[i] - last_mileage >= mileage_threshold)
        // || fabs(theta - last_theta) > math::D2R(20.0))
        {
            last_mileage = mileage_list[i];
            // last_theta = theta;

            //计算横向偏移量
            coef = (total_s - mileage_list[i]) / total_s;
            lat_offset = coef * coef * refer_line.second.dis;

            s.push_back(mileage_list[i]);
            x.push_back(center_line[i].x() - lat_offset * sin(theta));
            y.push_back(center_line[i].y() + lat_offset * cos(theta));
        }
    }
    if (fabs(s.back() - mileage_list.back()) > 0.1*mileage_list.back())
    {
        //计算横向偏移量
        coef = (total_s - mileage_list.back()) / total_s;
        lat_offset = coef * coef * refer_line.second.dis;

        theta = math::GetHeadingRadian(center_line.back().x() - center_line[num - 2].x(), 
                                       center_line.back().y() - center_line[num - 2].y());
        
        s.push_back(mileage_list.back());
        x.push_back(center_line.back().x() - lat_offset * sin(theta));
        y.push_back(center_line.back().y() + lat_offset * cos(theta));
    }

    //******spline拟合
    if (s.size() <= 2) 
        return false;
    spline_s_x.set_points(s, x);
    spline_s_y.set_points(s, y);
    predict_path.clear(); 

    double heading = 0;
    double ks = DBL_MIN;
    double x_s, y_s;
    double first_deriv_x, second_deriv_x, first_deriv_y, second_deriv_y;
    legionclaw::interface::PathPoint smooth_point;
    for (double cs = s[0];cs <= s.back();cs += delta_mileage)
    {
        spline_s_x.compute_spline(cs, x_s, first_deriv_x,
            second_deriv_x);  // x(s)一阶段,二阶段
        spline_s_y.compute_spline(cs, y_s, first_deriv_y,
                                    second_deriv_y);  // y(s)一阶段,二阶段
        heading = math::GetHeadingRadian(first_deriv_x, first_deriv_y);
        // ks = compute_ks_from_spline(first_deriv_x, second_deriv_x, first_deriv_y,
        // second_deriv_y);
        ks = math::CurveMath::ComputeCurvature(first_deriv_x, second_deriv_x,
                                                first_deriv_y, second_deriv_y);
        smooth_point.set_s(cs);
        smooth_point.set_theta(heading);
        smooth_point.set_kappa(ks);
        smooth_point.set_x(x_s);
        smooth_point.set_y(y_s);
        smooth_point.set_z(0.0);

        predict_path.push_back(smooth_point);
    }                                          

    return true;
}

/**
 * @brief Predict for vehicle with osm
 */
bool VectorMapPrediction::PredictVehicle(const lanelet::LaneletMapUPtr &map,
                                         const lanelet::routing::RoutingGraphUPtr &routing_map,
                                         const std::deque<legionclaw::interface::Obstacle> &object_deque,
                                         const PredictionConf &prediction_conf,
                                         const int &filter_mode, 
                                         std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    double predict_time = prediction_conf.prediction_horizon() * prediction_conf.predict_time_interval();
    double predict_time_interval =  prediction_conf.predict_time_interval();
    double filter_range =  prediction_conf.filter_range();
    int theta_mode = prediction_conf.theta_mode();
    double ele_diff_threshold = prediction_conf.ele_diff_threshold();
    // double begin_time=legionclaw::common::TimeTool::NowToSeconds();
    trajectories.clear();
    // std::vector<std::vector<PlannerHNS::WayPoint>> paths;
    legionclaw::interface::Obstacle object;
    if (object_deque.size() > 0)
        object = object_deque.front();

    double obj_v = hypot(object.velocity_abs().x(), object.velocity_abs().y());
    double obj_a = hypot(object.acceleration_abs().x(), object.acceleration_abs().y());
    if (isnan(obj_v) || isinf(obj_v))
        obj_v = 0.0;
    if (isnan(obj_a) || isinf(obj_a))
        obj_a = 0.0;
    // if (obj_v < prediction_conf.threshold_static_speed())
    //     return true;
    double prediction_distance = obj_v * predict_time; // + 0.5 * obj_a * predict_time * predict_time; //s=v*t+0.5*a*t^2
    double theta_start = object.theta_abs();
    //静止障碍物没必要通过速度计算方向角
    if (obj_v >= prediction_conf.threshold_static_speed() && (theta_mode == USE_VELOCITY_THETA || theta_mode == USE_MAP_THETA))
    // if (obj_v >= prediction_conf.threshold_static_speed() && theta_mode == USE_VELOCITY_THETA)
       theta_start = math::GetHeadingRadian(object.velocity_abs().x(), object.velocity_abs().y());
    // std::cout << "-----id : " << object.id() << ", -----velocity_abs().x() = : " << object.velocity_abs().x() << ", velocity_abs().y() = " << object.velocity_abs().y() << "\n"; 
    // std::cout << "0-----obj_v : " << obj_v << "\n"; 
    // std::cout << "1-----几何方向 : " << math::R2D(object.theta_abs()) << "\n"; 
    // std::cout << "2-----速度方向 : " << math::R2D(theta_start) << "\n";

    legionclaw::interface::TrajectoryPoint fake_pose;
    legionclaw::interface::PathPoint obstacle_point;
    obstacle_point.set_x(object.center_pos_abs().x());
    obstacle_point.set_y(object.center_pos_abs().y());
    obstacle_point.set_theta(theta_start);
    obstacle_point.set_kappa(0.0);
    obstacle_point.set_s(0.0);
    fake_pose.set_path_point(obstacle_point);
    fake_pose.set_v(obj_v);
    fake_pose.set_a(obj_a);    

    if (filter_mode == NO_MAP)
    {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }

    // std::vector<double> ClosestDistance;

    // std::vector<PlannerHNS::WayPoint *> closest_points = PlannerHNS::MappingHelpers::GetClosestWaypointsListFromMapWithGap(fake_pose, map, ClosestDistance,
    //                                                                                                                        lanelist_bak, 0, true);
    // //地图匹配成功
    lanelet::Point3d fake_point(lanelet::utils::getId(), object.center_pos_abs().x(), object.center_pos_abs().y(), object.center_pos_abs().z());
    // std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> refer_lines;
    // refer_lines = lanelet2_ex::GetRoute(map, routing_map, fake_point, v_theta, true, prediction_distance); //输出的中心线，待拟合
    std::vector<std::vector<std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr>>> refer_line_arry;
    bool in_lane = false;
    if(filter_mode == FILTER_BY_CENTER_POINT || filter_mode == FILTER_BY_POLYGON)
    {        
        // std::cout << "object.polygon_point_abs() : " << object.polygon_point_abs().size() << "\n";
        if(filter_mode == FILTER_BY_CENTER_POINT)
        {
            lanelet::Ids lane_ids = lanelet2_ex::GetClosestLaneletsInRange(map, fake_point, filter_range);
            if (lane_ids.size() > 0)
            {
                in_lane = true;
            }

        }
        else if(filter_mode == FILTER_BY_POLYGON)
        {
            for (auto polygon_point : object.polygon_point_abs())
            {
                lanelet::Point3d point(lanelet::utils::getId(), polygon_point.x(), polygon_point.y(), 0);
                lanelet::Ids lane_ids = lanelet2_ex::GetClosestLanelets(map, point);
                if (lane_ids.size() > 0)
                {
                    in_lane = true;
                    break;
                }
            }
        }        
        
        if (!in_lane)
        {
            return false;
        }
    }

    //静止障碍物不用再搜索路由
    if (obj_v < prediction_conf.threshold_static_speed())
        return true;

    refer_line_arry = lanelet2_ex::GetRoutes(map, routing_map, fake_point, 90.0 - math::R2D(theta_start), true,
                                             prediction_distance, ele_diff_threshold, in_lane); //write range into congig

    if (refer_line_arry.size() > 0)
    {
        // if (obj_v < prediction_conf.threshold_static_speed())
        //     return true;

        legionclaw::prediction::PredictionTrajectory trajectory;
        legionclaw::interface::TrajectoryInPrediction predict_trajectory;
        legionclaw::interface::PathPoint end_point;
        double threshold_dis = 3.0;
        std::vector<double> scores, probilities;
        for (unsigned int i = 0; i < refer_line_arry.size(); i++)
        {
            for (unsigned int j = 0; j < refer_line_arry[i].size(); j++)
            {
                auto raw_line = refer_line_arry[i][j];
                std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr> refer_line;
                double dis = 0.0;
                refer_line.first.clear();
                refer_line.second = raw_line.second;
                // for (auto raw_point : raw_line.first)
                legionclaw::interface::Point3D raw_point;
                std::vector<double> mileage_list;
                for (unsigned int k = raw_line.second.index; k < raw_line.first.size(); k++)
                {
                    raw_point = raw_line.first.at(k);
                    if (!refer_line.first.empty())
                        dis += math::Norm(raw_point.x() - refer_line.first.back().x(), raw_point.y() - refer_line.first.back().y());
                    if (dis > prediction_distance && mileage_list.size() >= 5)
                        break;
                    mileage_list.push_back(dis);
                    refer_line.first.push_back(raw_point);
                }
                int line_num = refer_line.first.size();
                if (line_num > 1)
                {
                    if (theta_mode == USE_MAP_THETA)
                    {
                        double heading_start = math::GetHeading(refer_line.first.front().x(), refer_line.first.front().y(),
                                                            refer_line.first.at(1).x(), refer_line.first.at(1).y());
                        obstacle_point.set_theta(math::NormalizeAngle(math::D2R(90.0 - heading_start)));
                    }

                    /*************Spline方法生成预测轨迹***************/
                    // std::vector<legionclaw::interface::Point3D> center_line = refer_line.first;
                    double delta_mileage = predict_time_interval * obj_v;
                    std::vector<legionclaw::interface::PathPoint> predict_path;
                    if (!GeneratePath(refer_line, mileage_list, obstacle_point, delta_mileage, predict_path))
                        continue;

                    /*************Spiral方法生成预测轨迹***************/
                    // double heading_end = math::GetHeading(refer_line.first.at(line_num - 2).x(), refer_line.first.at(line_num - 2).y(),
                    //                                       refer_line.first.back().x(), refer_line.first.back().y());

                    // // double heading_start = math::GetHeading(refer_line.first.at(0).x(), refer_line.first.at(0).y(),
                    // //                                     refer_line.first.at(1).x(), refer_line.first.at(1).y());

                    // // double deta_theta = math::AngleDiff(obstacle_point.theta(), math::D2R(90.0 - heading_start));

                    // end_point.set_x(refer_line.first.back().x());
                    // end_point.set_y(refer_line.first.back().y());
                    // end_point.set_theta(math::NormalizeAngle(math::D2R(90.0 - heading_end)));
                    // end_point.set_kappa(0.0);

                    // //Spiral
                    // std::vector<legionclaw::interface::PathPoint> predict_path;
                    // spiral::CubicSpiralCurve csc(obstacle_point, end_point);
                    // csc.SetSpiralConfig(spiral_params_);
                    // csc.CalculatePath();
                    // // double dx = end_point.x() - obstacle_point.x();
                    // // double dy = end_point.y() - obstacle_point.y();
                    // int num = (int)(predict_time / predict_time_interval);
                    // csc.GetPathVec(num, &predict_path);

                    if (predict_path.empty())
                        continue;

                    legionclaw::interface::TrajectoryPoint trajectory_point;
                    predict_trajectory.clear_trajectory_points();
                    double average_kappa = 0.0;
                    double average_kappa_abs = 0.0,average_kappa_max = 0.0;
                    unsigned int kappa_size = 0;
                    std::pair<double, int> max_kappa = {0.0, -1};
                    for (unsigned int i = 0; i < predict_path.size(); i++)
                    {
                        trajectory_point.set_path_point(predict_path.at(i));
                        
                        trajectory_point.set_relative_time(predict_time_interval * i);
                        trajectory_point.set_v(obj_v);
                        trajectory_point.set_a(obj_a);

                        average_kappa += predict_path.at(i).kappa();
                        double kappa_abs = fabs(predict_path.at(i).kappa());
                        average_kappa_abs += kappa_abs;
                        if (kappa_abs > prediction_conf.road_curvature_threshold())
                        {
                            average_kappa_max += kappa_abs;
                            kappa_size++;
                        }
                        
                        predict_trajectory.add_trajectory_points(trajectory_point);

                        if(kappa_abs > max_kappa.first)
                        {
                            max_kappa.first = kappa_abs;
                            max_kappa.second = i;
                        }
                    }
                    // std::cout << "------------ max_kappa.first = " << max_kappa.first << "\n";
                    if (fabs(max_kappa.first) > 0.5)
                        average_kappa = max_kappa.first;
                    else
                        average_kappa /= predict_path.size();

                    average_kappa_abs /= predict_path.size();
                    if (kappa_size)
                    {
                        average_kappa_max /= kappa_size;
                    }

                    //2state spline
                    // spline::CubicSpline spline_x, spline_y;
                    // double theta_start = math::D2R(90.0 - heading_start);
                    // spline_x.set_points(0.0, obstacle_point.x(),
                    //                                     predict_time,  end_point.x());
                    // spline_x.set_boundary(obj_v*cos(theta_start), obj_v*cos(end_point.theta()));
                    // spline_x.compute_coef();

                    // spline_y.set_points(0.0, obstacle_point.y(),
                    //                                     predict_time,  end_point.y());
                    // spline_y.set_boundary(obj_v*sin(theta_start), obj_v*sin(end_point.theta()));
                    // spline_y.compute_coef();

                    // legionclaw::interface::PathPoint predict_point;
                    // legionclaw::interface::TrajectoryPoint trajectory_point;
                    // for (double t = 0.0; t <= predict_time; t += predict_time_interval)
                    // {
                    //     predict_point.set_x(spline_x(t));
                    //     predict_point.set_y(spline_y(t));
                    //     predict_point.set_s(obj_v * t);
                    //     trajectory_point.set_path_point(predict_point);

                    //     trajectory_point.set_relative_time(t);
                    //     trajectory_point.set_v(obj_v);
                    //     trajectory_point.set_a(obj_a);
                    //     predict_trajectory.add_trajectory_points(trajectory_point);
                    // }

                    trajectory.set_trajectory_in_prediction(predict_trajectory);
                    trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VECTORMAP);
                    switch (refer_line.second.dir)
                    {
                    case lanelet2_ex::STRAIGHT:
                        trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_STRAIGHT);
                        break;
                    case lanelet2_ex::LEFT:
                        trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_LEFT);
                        break;
                    case lanelet2_ex::RIGHT:
                        trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_RIGHT);
                        break;
                    default:
                        break;
                    }

                    double score_state = threshold_dis - fabs(refer_line.second.dis) - 3.0 * (fabs(math::NormalizeAngle(math::D2R(refer_line.second.theta_diff))) - M_PI_2);
                    double score_theta_diff = 4.0 * fabs(math::AngleDiff(theta_start, 
                                    predict_trajectory.trajectory_points().front().path_point().theta()));
                    // double score_kappa = average_kappa;
                    if(obj_v > prediction_conf.dynamic_obstacle_speed_threshold())
                    {
                        if(average_kappa_abs < prediction_conf.road_curvature_threshold())
                            score_state += 15; //增加速度较快时直行的权重，exp(15)=3269017.3724721107
                        else
                            score_state -= 15; //减小速度较快时曲线的权重
                    }
                    
                    double tmp_score = score_state - fabs(5 * average_kappa) - score_theta_diff;
                    // std::cout<<"object_vehicle.id = "<<object.id()<<", object.v = "<<obj_v<<", score = "<<tmp_score<<", average_kappa_abs = "
                    //         <<average_kappa_abs<<", average_kappa_max = "<<average_kappa_max<<", average_kappa = "<<average_kappa<<"\n";
                    scores.push_back(tmp_score);
                    // if (object.id() == 11492)
                    // std::cout << "******** score_state = " << score_state << ", average_kappa = " << average_kappa*5
                    //           << "score_theta_diff = " << score_theta_diff << "\n";
                    // std::cout << "dis = " << refer_line.second.dis << ", theta_diff = " << math::NormalizeAngle(math::D2R(refer_line.second.theta_diff)) 
                    //           << ", score = " << threshold_dis - fabs(refer_line.second.dis) - 5.0 * (fabs(math::NormalizeAngle(math::D2R(refer_line.second.theta_diff))) - M_PI_2) << "\n";
                    trajectories.push_back(trajectory);
                }
            }
        }
        
        ComputeProbability(scores, probilities);
        if (probilities.size() != trajectories.size())
        {
            std::cout<<"probilities.size() != trajectories.size()"<<"\n";
            return false;
        }

        for (unsigned int i = 0; i < probilities.size(); i++)
        {
            trajectories.at(i).mutable_trajectory_in_prediction()->set_probability(probilities.at(i));
        }
    }
    else
    {
        // if (obj_v < prediction_conf.threshold_static_speed())
        //     return true;
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }
    // double pred_time=legionclaw::common::TimeTool::NowToSeconds();
    // std::cout<<"222222222222222222222pred_time  :   "<<pred_time-route_time<<"     seconds "<<"\n"<<"\n";

    return true;
}

/**
 * @brief Predict for pedestrian with localmap
 */
bool VectorMapPrediction::PredictPedestrian(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines,
                                            const std::deque<legionclaw::interface::Obstacle> &object_deque,
                                            const PredictionConf &prediction_conf,
                                            const int &filter_mode, 
                                            std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    // double predict_time = prediction_conf.prediction_horizon() * prediction_conf.predict_time_interval();
    double predict_time = 1.0;  //对于行人、自行车等，预测时间由4s改为1s
    double predict_time_interval =  prediction_conf.predict_time_interval();
    // double filter_range =  prediction_conf.filter_range();
    int theta_mode = prediction_conf.theta_mode();

    trajectories.clear();
    legionclaw::interface::Obstacle object;
    if (object_deque.size() > 0)
        object = object_deque.front();

    double obj_v = hypot(object.velocity_abs().x(), object.velocity_abs().y());
    double obj_a = hypot(object.acceleration_abs().x(), object.acceleration_abs().y());
    if (isnan(obj_v) || isinf(obj_v))
        obj_v = 0.0;
    if (isnan(obj_a) || isinf(obj_a))
        obj_a = 0.0;
    // double prediction_distance = obj_v * predict_time; // + 0.5 * obj_a * predict_time * predict_time; //s=v*t+0.5*a*t^2
    double theta_start = object.theta_abs();
    //静止障碍物没必要通过速度计算方向角
    if (obj_v >= prediction_conf.threshold_static_speed() && (theta_mode == USE_VELOCITY_THETA || theta_mode == USE_MAP_THETA))
       theta_start = math::GetHeadingRadian(object.velocity_abs().x(), object.velocity_abs().y());

    legionclaw::interface::TrajectoryPoint fake_pose;
    legionclaw::interface::PathPoint obstacle_point;
    obstacle_point.set_x(object.center_pos_abs().x());
    obstacle_point.set_y(object.center_pos_abs().y());
    obstacle_point.set_theta(theta_start);
    obstacle_point.set_kappa(0.0);
    obstacle_point.set_s(0.0);
    fake_pose.set_path_point(obstacle_point);
    fake_pose.set_v(obj_v);
    fake_pose.set_a(obj_a);

    // 目前将自行车行人等障碍物预测轨迹改为直线预测
    if (obj_v >= prediction_conf.threshold_static_speed())
    {
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }
    return true;
    /* 
    if (filter_mode == NO_MAP)
    {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }

    //判断障碍物中心点是否在车道内in_lane
    bool in_lane = false;
    int min_lane_id = -1;
    int min_match_indx = -1;
    // std::vector<int> match_indxs;
    double min_match_dis = DBL_MAX;
    // std::vector<double> match_diss;
    double match_dis = DBL_MAX;
    for (unsigned int i = 0; i < smooth_lines.size(); i++) {
        if (smooth_lines.size() < 1) {
            AERROR << "smooth_lines.size() < 1";
            return false;
        }

        int match_index = MapMatcher::QueryNearestPointWithBuffer(
                            smooth_lines.at(i), {obstacle_point.x(), obstacle_point.y()},
                           1.0e-6, match_dis);

        if (fabs(match_dis) < fabs(min_match_dis)) {
            min_match_dis = match_dis;
            min_lane_id = i;
            min_match_indx = match_index;
        }
        // match_indxs.push_back(match_index);
        // match_diss.push_back(match_dis);
    }
    // 匹配完所有车道
    if (min_match_indx > -1)
    {
        //左+右-
        double filter_range_to_center = filter_range;
        if (match_dis > 0)
            filter_range_to_center += smooth_lines.at(min_lane_id).at(min_match_indx).left_road_width();
        else
            filter_range_to_center += smooth_lines.at(min_lane_id).at(min_match_indx).right_road_width();

        if (fabs(min_match_dis) <= filter_range_to_center )
        {
            in_lane = true;
        }
    }

    if(filter_mode && !in_lane) //如果需要过滤（暂时不区分中心点过滤还是polygon过滤）
        return false;

    //加入theta进一步与参考线匹配，如果匹配成功（theta<45）使用参考线生成轨迹，匹配失败则直线运动
    // for (unsigned int i = 0; i < smooth_lines.size(); i++)
    // {
    //     int match_index = MapMatcher::QueryVerticalDistanceWithBuffer(
    //                         smooth_lines.at(i), {obstacle_point.x(), obstacle_point.y()}, obstacle_point.theta(),
    //                        1.0e-6, match_dis);

    //     if (fabs(match_dis) < fabs(min_match_dis)) {
    //         min_match_dis = match_dis;
    //         min_lane_id = i;
    //         min_match_indx = match_index;
    //     }
    // }
    //暂时直接比较theta
    if (min_lane_id < 0 || smooth_lines.size() == 0)
    {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }
    auto ref_line = smooth_lines[min_lane_id];
    double theta_diff = fabs(math::AngleDiff(obstacle_point.theta(), ref_line[min_match_indx].theta()));
    if (theta_diff < 0.25 * M_PI)
    {
        if (obj_v < prediction_conf.threshold_static_speed())
            return true;

        legionclaw::prediction::PredictionTrajectory trajectory;
        legionclaw::interface::TrajectoryInPrediction predict_trajectory;
        legionclaw::interface::PathPoint start_point, end_point;

        start_point.set_x(ref_line[min_match_indx].point().x());
        start_point.set_y(ref_line[min_match_indx].point().y());
        start_point.set_theta(ref_line[min_match_indx].theta());
        start_point.set_kappa(0.0);

        double start_s = ref_line[min_match_indx].mileage();
        double end_s = start_s + prediction_distance;
        auto end_state = MapMatcher::QueryNearestPoint(ref_line, end_s);
        end_point.set_x(end_state.point().x());
        end_point.set_y(end_state.point().y());
        end_point.set_theta(end_state.theta());
        end_point.set_kappa(0.0);

        std::vector<legionclaw::interface::PathPoint> ref_path, predict_path;
        spiral::CubicSpiralCurve csc(start_point, end_point);
        csc.SetSpiralConfig(spiral_params_);
        csc.CalculatePath();
        double dx = end_point.x() - start_point.x();
        double dy = end_point.y() - start_point.y();
        int num = (int)(predict_time / predict_time_interval);
        csc.GetPathVec(num, &ref_path);
        // int64_t t5 = TimeTool::Now2Us();  //for test

        if (ref_path.empty())
            return false;

        //translate
        TranslateLine(ref_path, 0, min_match_dis, predict_path);

        //path 2 trajectory
        legionclaw::interface::TrajectoryPoint trajectory_point;
        predict_trajectory.clear_trajectory_points();
        for (unsigned int i = 0; i < predict_path.size(); i++)
        {
            trajectory_point.set_path_point(predict_path.at(i));
            
            trajectory_point.set_relative_time(predict_time_interval * i);
            trajectory_point.set_v(obj_v);
            trajectory_point.set_a(obj_a);
            
            predict_trajectory.add_trajectory_points(trajectory_point);
            predict_trajectory.set_probability(1.0);
        }
        trajectory.set_trajectory_in_prediction(predict_trajectory);
        trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VECTORMAP);
        trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_STRAIGHT);
        trajectories.push_back(trajectory);
    }
    else
    {
        if (obj_v < prediction_conf.threshold_static_speed())
            return true;
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }

    return true;
    */
}

/**
 * @brief Predict for vehicle with localmap
 */
bool VectorMapPrediction::PredictVehicle(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines,
                                         const std::deque<legionclaw::interface::Obstacle> &object_deque,
                                         const PredictionConf &prediction_conf,
                                         const int &filter_mode, 
                                         std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    double predict_time = prediction_conf.prediction_horizon() * prediction_conf.predict_time_interval();
    double predict_time_interval =  prediction_conf.predict_time_interval();
    double filter_range =  prediction_conf.filter_range();
    int theta_mode = prediction_conf.theta_mode();
    
    trajectories.clear();
    legionclaw::interface::Obstacle object;
    if (object_deque.size() > 0)
        object = object_deque.front();

    double obj_v = hypot(object.velocity_abs().x(), object.velocity_abs().y());
    double obj_a = hypot(object.acceleration_abs().x(), object.acceleration_abs().y());
    if (isnan(obj_v) || isinf(obj_v))
        obj_v = 0.0;
    if (isnan(obj_a) || isinf(obj_a))
        obj_a = 0.0;
    double prediction_distance = obj_v * predict_time; // + 0.5 * obj_a * predict_time * predict_time; //s=v*t+0.5*a*t^2
    double theta_start = object.theta_abs();
    //静止障碍物没必要通过速度计算方向角
    if (obj_v >= prediction_conf.threshold_static_speed() && (theta_mode == USE_VELOCITY_THETA || theta_mode == USE_MAP_THETA))
       theta_start = math::GetHeadingRadian(object.velocity_abs().x(), object.velocity_abs().y());

    legionclaw::interface::TrajectoryPoint fake_pose;
    legionclaw::interface::PathPoint obstacle_point;
    obstacle_point.set_x(object.center_pos_abs().x());
    obstacle_point.set_y(object.center_pos_abs().y());
    obstacle_point.set_theta(theta_start);
    obstacle_point.set_kappa(0.0);
    obstacle_point.set_s(0.0);
    fake_pose.set_path_point(obstacle_point);
    fake_pose.set_v(obj_v);
    fake_pose.set_a(obj_a);    

    if (filter_mode == NO_MAP)
    {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }

    if (smooth_lines.size() < 1 && filter_mode == FILTER_NONE) {
        if (obj_v >= prediction_conf.threshold_static_speed())
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
        return true;
    }
    else if(smooth_lines.size() < 1 && filter_mode) 
        return false;

    //判断障碍物中心点是否在车道内
    bool in_lane = false;
    int min_lane_id = -1;
    int min_match_indx = -1;
    std::vector<int> match_indxs;
    double min_match_dis = DBL_MAX;
    std::vector<double> match_diss;
    double match_dis = DBL_MAX;
    for (unsigned int i = 0; i < smooth_lines.size(); i++) {
        int match_index = MapMatcher::QueryNearestPointWithBuffer(
                            smooth_lines.at(i), {obstacle_point.x(), obstacle_point.y()},
                           1.0e-6, match_dis);

        if (fabs(match_dis) < fabs(min_match_dis)) {
            min_match_dis = match_dis;
            min_lane_id = i;
            min_match_indx = match_index;
        }
        match_indxs.push_back(match_index);
        match_diss.push_back(match_dis);
    }
    // 匹配完所有车道
    if (min_match_indx > -1)
    {
        //左+右-
        double filter_range_to_center = filter_range;
        if (min_match_dis > 0)
            filter_range_to_center += smooth_lines.at(min_lane_id).at(min_match_indx).left_road_width();
        else
            filter_range_to_center += smooth_lines.at(min_lane_id).at(min_match_indx).right_road_width();

        if (fabs(min_match_dis) <= filter_range_to_center )
        {
            in_lane = true;
        }
    }

    if(filter_mode && !in_lane) //如果需要过滤（暂时不区分中心点过滤还是polygon过滤）
        return false;

    if (in_lane)    //如果在车道内，用smooth_lines拟合预测轨迹
    {
        if (obj_v < prediction_conf.threshold_static_speed())
            return true;

        legionclaw::prediction::PredictionTrajectory trajectory;
        legionclaw::interface::TrajectoryInPrediction predict_trajectory;
        legionclaw::interface::PathPoint end_point;
        double threshold_dis = 3.0;
        std::vector<double> scores, probilities;
        for (unsigned int i = 0; i < smooth_lines.size(); i++)
        {
            if (match_indxs[i] < 0) //没匹配上的车道，不进行预测
                continue;
            double start_s = smooth_lines[i][match_indxs[i]].mileage();
            double remain_s = smooth_lines[i].back().mileage() - start_s;
            if (remain_s < 0.8 * prediction_distance || //如果剩下的距离不够
                 fabs(match_diss[i]) > smooth_lines[i][match_indxs[i]].left_road_width())   //横向距离超过半个车道
            {
                // PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
                // return true;
                continue;
            }
            double end_s = start_s + prediction_distance;
            auto end_state = MapMatcher::QueryNearestPoint(smooth_lines[i], end_s);
            if (theta_mode == USE_MAP_THETA)
            {
                obstacle_point.set_theta(smooth_lines[i][match_indxs[i]].theta());
            }
            end_point.set_x(end_state.point().x());
            end_point.set_y(end_state.point().y());
            end_point.set_theta(end_state.theta());
            end_point.set_kappa(0.0);

            //Spiral
            std::vector<legionclaw::interface::PathPoint> predict_path;
            spiral::CubicSpiralCurve csc(obstacle_point, end_point);
            csc.SetSpiralConfig(spiral_params_);
            csc.CalculatePath();
            // double dx = end_point.x() - obstacle_point.x();
            // double dy = end_point.y() - obstacle_point.y();
            int num = (int)(predict_time / predict_time_interval);
            csc.GetPathVec(num, &predict_path);

            if (predict_path.empty())
                continue;

            legionclaw::interface::TrajectoryPoint trajectory_point;
            predict_trajectory.clear_trajectory_points();
            for (unsigned int i = 0; i < predict_path.size(); i++)
            {
                trajectory_point.set_path_point(predict_path.at(i));
                
                trajectory_point.set_relative_time(predict_time_interval * i);
                trajectory_point.set_v(obj_v);
                trajectory_point.set_a(obj_a);
                
                predict_trajectory.add_trajectory_points(trajectory_point);
            }

            trajectory.set_trajectory_in_prediction(predict_trajectory);
            trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VECTORMAP);

            double theta_diff = fabs(math::AngleDiff(obstacle_point.theta(), smooth_lines[i][match_indxs[i]].theta()));
            scores.push_back(threshold_dis - fabs(match_diss[i]) - 3.0 * (theta_diff - M_PI_2));
            trajectories.push_back(trajectory);
        
        }
        if (trajectories.size() == 0)
        {
            PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
            return true;
        }
        ComputeProbability(scores, probilities);
        if (probilities.size() != trajectories.size())
            return false;
        for (unsigned int i = 0; i < probilities.size(); i++)
        {
            trajectories.at(i).mutable_trajectory_in_prediction()->set_probability(probilities.at(i));
        }
    }
    else    //如果不在车道内，直线预测轨迹
    {
        if (obj_v < prediction_conf.threshold_static_speed())
            return true;
        PredictTrajectoriesUsingVelocityWithT(fake_pose, predict_time, predict_time_interval, prediction_conf.threshold_static_speed(), trajectories);
    }

    return true;
}

/*************************************************************************************************************************************/
bool VectorMapPrediction::ComputeProbability(const std::vector<double> &scores, std::vector<double> &probilities)
{
    double sum = 0.0;
    std::vector<double> exs;
    //exp
    for (auto score : scores)
    {
        double ex = exp(score);
        sum += ex;
        exs.push_back(ex);
    }

    //normalize and compute probilities
    for (auto ex : exs)
    {
        // double norm = ex / sum;
        // double probility = -log(norm);
        // probilities.push_back(probility);
        probilities.push_back(ex / sum);

        // if (isnan(ex / sum))
        // {
        //     std::cout << ex << " xxxxxxxxxxxxxxxxxxxxxxx " << sum << "\n";
        //     for (auto score : scores)
        //         std::cout << "--------" << score << "\n";
        // }
    }

    return true;
}

bool VectorMapPrediction::TranslateLine(const std::vector<legionclaw::interface::PathPoint> &raw_line,
                                        const int &start_index, const double &lat_offset, //left - right +
                                        std::vector<legionclaw::interface::PathPoint> &translated_line)
{
    if (start_index < 0 || start_index > int(raw_line.size() - 1))
        return false;

    legionclaw::interface::PathPoint tmp_point;
    translated_line.clear();

    for (unsigned int i = start_index + 1; i < raw_line.size(); ++i)
    {
        tmp_point.set_x(raw_line[i].x() - lat_offset * sin(raw_line[i].theta()));
        tmp_point.set_y(raw_line[i].y() + lat_offset * cos(raw_line[i].theta()));
        tmp_point.set_theta(raw_line[i].theta());
        if (raw_line[i].kappa() == 0.0)
            tmp_point.set_kappa(0.0);
        else
            tmp_point.set_kappa(1.0 / (1.0 / raw_line[i].kappa() + lat_offset));
        tmp_point.set_s(raw_line[i].s() - raw_line[start_index].s());

        translated_line.push_back(tmp_point);
    }
    return true;
}

bool VectorMapPrediction::PredictTrajectoriesUsingVelocity(const legionclaw::interface::TrajectoryPoint &start_pose,
                                                           const double &max_predict_distance, const double trajectory_density,
                                                           const double &threshold_static_speed,
                                                           std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    if (start_pose.v() < threshold_static_speed)
        return false;
    legionclaw::prediction::PredictionTrajectory trajectory;
    legionclaw::interface::TrajectoryInPrediction predict_trajectory;
    legionclaw::interface::TrajectoryPoint tp = start_pose;
    legionclaw::interface::PathPoint wp = start_pose.path_point();
    tp.set_relative_time(0.0);

    predict_trajectory.add_trajectory_points(tp);
    predict_trajectory.set_probability(1.0);

    double dx = trajectory_density * cos(start_pose.path_point().theta());
    double dy = trajectory_density * sin(start_pose.path_point().theta());
    double dt = trajectory_density / start_pose.v();

    for (double dlength = trajectory_density; dlength <= max_predict_distance; dlength += trajectory_density)
    {
        // std::cout << "====== max_predict_distance = " << max_predict_distance << ", start_pose.v() = " << start_pose.v() << "\n";
        wp.set_x(wp.x() + dx);
        wp.set_y(wp.y() + dy);
        wp.set_theta(wp.theta());
        wp.set_s(wp.s() + trajectory_density);
        tp.set_path_point(wp);
        tp.set_v(tp.v());
        tp.set_a(tp.a());
        tp.set_relative_time(tp.relative_time() + dt);
        predict_trajectory.add_trajectory_points(tp);
    }

    trajectory.set_trajectory_in_prediction(predict_trajectory);
    trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VELOCITY);
    trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_STRAIGHT);

    trajectories.push_back(trajectory);

    return true;
}

bool VectorMapPrediction::PredictTrajectoriesUsingVelocityWithT(const legionclaw::interface::TrajectoryPoint &start_pose,
                                                           const double &predict_time, const double time_interval,
                                                           const double &threshold_static_speed,
                                                           std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories)
{
    if (start_pose.v() < threshold_static_speed)
        return false;
    legionclaw::prediction::PredictionTrajectory trajectory;
    legionclaw::interface::TrajectoryInPrediction predict_trajectory;
    legionclaw::interface::TrajectoryPoint tp = start_pose;
    legionclaw::interface::PathPoint wp = start_pose.path_point();
    tp.set_relative_time(0.0);

    predict_trajectory.add_trajectory_points(tp);
    predict_trajectory.set_probability(1.0);

    double v = start_pose.v();
    double dx = v * time_interval * cos(start_pose.path_point().theta());
    double dy = v * time_interval * sin(start_pose.path_point().theta());

    for (double t = time_interval; t <= predict_time; t += time_interval)
    {
        wp.set_x(wp.x() + dx);
        wp.set_y(wp.y() + dy);
        wp.set_theta(wp.theta());
        wp.set_s(wp.s() + v * time_interval);
        tp.set_path_point(wp);
        tp.set_v(tp.v());
        tp.set_a(tp.a());
        tp.set_relative_time(t);
        predict_trajectory.add_trajectory_points(tp);
    }

    trajectory.set_trajectory_in_prediction(predict_trajectory);
    trajectory.set_predict_mode(legionclaw::prediction::PREDICT_MODE_VELOCITY);
    trajectory.set_predict_direction(legionclaw::prediction::PREDICT_DIR_STRAIGHT);

    trajectories.push_back(trajectory);

    return true;
}
