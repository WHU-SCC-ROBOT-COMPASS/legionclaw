#pragma once
#include <deque>

#include "modules/common/interface/obstacle.hpp"
#include "modules/prediction/src/interface/prediction_trajectory.hpp"
// #include "RoadNetwork.h"
// #include "common/map_api/map_convert/include/map_convert.h"
#include "lanelet2_ex.h"
#include "modules/prediction/src/common/spiral_curve/cubic_spiral_curve.h"
// #include "modules/prediction/src/common/spiral_curve/quintic_spiral_curve.h"
// #include "modules/prediction/src/common/spline/cubic_spline.h"
#include "modules/prediction/src/common/spline/spline.h"
#include "modules/common/math/vec2d.h"

using namespace legionclaw::prediction;
using namespace legionclaw::common;
// using namespace lanelet2_ex;


class VectorMapPrediction
{

public:
    VectorMapPrediction()
    {
        //TODO:add config
        spiral_params_.simpson_size = 9;
        spiral_params_.newton_raphson_tol = 0.01;
        spiral_params_.newton_raphson_max_iter = 20;
    }

    // VectorMapPrediction(const double &param_prediction_horizon, const double &param_history_horizon, const double &param_predict_traj_num);

    ~VectorMapPrediction()
    {
    }

    /**
     * @brief Predict for pedestrian with osm
     */
    bool PredictPedestrian(const lanelet::LaneletMapUPtr &map,
                           const lanelet::routing::RoutingGraphUPtr &routing_map,
                           const std::deque<legionclaw::interface::Obstacle> &object_deque,
                           const PredictionConf &prediction_conf,
                           const int &filter_mode, 
                           std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);

    bool GeneratePath(const std::vector<legionclaw::interface::Point3D> &center_line,
                const std::vector<double> &mileage_list,
                const legionclaw::interface::PathPoint &start_point,
                const double &delta_mileage,
                std::vector<legionclaw::interface::PathPoint> &predict_path);
    bool GeneratePath(const std::pair<std::vector<legionclaw::interface::Point3D>, lanelet2_ex::Lan_Atr> &refer_line,
                const std::vector<double> &mileage_list,
                const legionclaw::interface::PathPoint &start_point,
                const double &delta_mileage,
                std::vector<legionclaw::interface::PathPoint> &predict_path);

    /**
     * @brief Predict for vehicle with osm
     */
    bool PredictVehicle(const lanelet::LaneletMapUPtr &map,
                        const lanelet::routing::RoutingGraphUPtr &routing_map,
                        const std::deque<legionclaw::interface::Obstacle> &object_deque,
                        const PredictionConf &prediction_conf,
                        const int &filter_mode, 
                        std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);

    /**
     * @brief Predict for pedestrian with localmap
     */
    bool PredictPedestrian(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines,
                           const std::deque<legionclaw::interface::Obstacle> &object_deque,
                           const PredictionConf &prediction_conf,
                           const int &filter_mode, 
                           std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);

    /**
     * @brief Predict for vehicle with localmap
     */
    bool PredictVehicle(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines,
                        const std::deque<legionclaw::interface::Obstacle> &object_deque,
                        const PredictionConf &prediction_conf,
                        const int &filter_mode, 
                        std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);

    bool ComputeProbability(const std::vector<double> &scores, std::vector<double> &probilities);

    bool TranslateLine(const std::vector<legionclaw::interface::PathPoint> &raw_line,
                       const int &start_index, const double &lat_offset, //left - right +
                       std::vector<legionclaw::interface::PathPoint> &translated_line);

    /**
     * @brief Predict trajectory using const velocity 
     */
    bool PredictTrajectoriesUsingVelocity(const legionclaw::interface::TrajectoryPoint &start_pose,
                                          const double &max_predict_distance, const double trajectory_density,
                                          const double &threshold_static_speed,
                                          std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);

    bool PredictTrajectoriesUsingVelocityWithT(const legionclaw::interface::TrajectoryPoint &start_pose,
                                            const double &predict_time, const double time_interval,
                                            const double &threshold_static_speed,
                                            std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories);
                                            
private:
    //self pose
    double curr_pose_x_ = 0.0;
    double curr_pose_y_ = 0.0;
    // map_convert::Converter map_c;
    // lanelet::traffic_rules::TrafficRulesPtr trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    // lanelet::routing::RoutingGraphUPtr graph;
    bool b_generate_branches_ = false;

    spiral::SpiralCurveConfig spiral_params_; //螺旋曲线参数
};
