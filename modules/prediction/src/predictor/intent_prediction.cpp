#include "modules/common/math/math_utils.h"
#include "modules/common/math/math_tools.h"
#include "modules/prediction/src/common/enum_list.h"
//#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/prediction/src/predictor/intent_prediction.h"
#include <math.h>

using namespace legionclaw::common;
using namespace legionclaw::interface;
using namespace legionclaw::prediction;
// using namespace math;
/**
 * @brief Predict for pedestrian
 */
bool IntentPrediction::PredictIntentForPedestrian(const lanelet::LaneletMapUPtr &map,
                                                  const std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories,
                                                  const double &predict_time,
                                                  legionclaw::interface::ObstacleIntent &intent)
{
    if (trajectories.size() == 0)
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_STOPPING_STATE);
        return true;
    }

    /*取出概率最大的轨迹*/
    double max_probility = -1;
    int max_index = -1;
    for (unsigned int ip = 0; ip < trajectories.size(); ip++)
    {
        if (trajectories.at(ip).trajectory_in_prediction().probability() > max_probility)
        {
            max_probility = trajectories.at(ip).trajectory_in_prediction().probability();
            max_index = ip;
        }
    }
    if (max_index < 0 || max_index > int(trajectories.size()) || max_probility < 0)
        return false;

    // std::vector<legionclaw::interface::TrajectoryPointInPrediction> intent_points;
    // trajectories.at(max_index).trajectory_in_prediction().trajectory_points(intent_points);
    legionclaw::prediction::PredictionTrajectory intent_trajectory;
    intent_trajectory = trajectories.at(max_index);

    if (intent_trajectory.trajectory_in_prediction().trajectory_points_size() == 0)
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_STOPPING_STATE);
        return true;
    }

    /*横穿马路/沿路行走*/
    unsigned int sample_num = 5;                                                                                       //intent_trajectory.trajectory_points().size();//5;
    int step_num = (int)(intent_trajectory.trajectory_in_prediction().trajectory_points_size() / sample_num); //初始化计算步长
    if (step_num == 0)
    {
        sample_num = intent_trajectory.trajectory_in_prediction().trajectory_points_size();
        step_num = 1;
    }

    if (intent_trajectory.predict_mode() == legionclaw::prediction::PREDICT_MODE_VELOCITY)
    {
        bool match_valid = false;
        std::pair<lanelet::ConstPoint3d, double> match_path_point;
        double px = 0.0, py = 0.0, ptheta = 0.0, yaw_diff = 0.0;
        int match_valid_count = 0;
        for (unsigned int sample_i = 0; sample_i < sample_num; sample_i += step_num)
        {
            px = intent_trajectory.trajectory_in_prediction().trajectory_points(sample_i).path_point().x();
            py = intent_trajectory.trajectory_in_prediction().trajectory_points(sample_i).path_point().y();
            ptheta = intent_trajectory.trajectory_in_prediction().trajectory_points(sample_i).path_point().theta();
            lanelet::Point3d point(lanelet::utils::getId(), px, py, 0);
            point.x() =
                match_valid = lanelet2_ex::GetClosestPathPointInRange(map, point, math::R2D(ptheta), 5.0, match_path_point);
            if (match_valid)
            {
                match_valid_count++;
                yaw_diff += fabs(math::AngleDiff(math::D2R(match_path_point.second), ptheta));
            }
        }

        yaw_diff /= match_valid_count;
        if (yaw_diff > 0.05 * M_PI && yaw_diff < 0.75 * M_PI)
        {
             intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_CROSS_ROAD_STATE);
            return true;
        }
        else if (yaw_diff < 0.1 * M_PI)
        {
             intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_ALONG_ROAD_STATE);
            return true;
        }
        else if (yaw_diff > 0.85 * M_PI)
        {
             intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_ONCOMMING_STATE);
            return true;
        }
    }

    /*匀速、加速、减速*/
    double acceleration = 0.0;
    double dt = 0.0;
    double accumulate_time_acc = 0.0;
    double accumulate_time_dec = 0.0;
    double accumulate_time_keep = 0.0;
    double accumulate_time_stop = 0.0;
    //暂时不考虑连续性
    for (unsigned int i = 0; i < intent_trajectory.trajectory_in_prediction().trajectory_points_size() - 1; ++i)
    {
        dt = intent_trajectory.trajectory_in_prediction().trajectory_points(i + 1).relative_time() -
             intent_trajectory.trajectory_in_prediction().trajectory_points(i).relative_time();
        acceleration = (intent_trajectory.trajectory_in_prediction().trajectory_points(i + 1).v() -
                        intent_trajectory.trajectory_in_prediction().trajectory_points(i).v()) /
                       dt;
        if (acceleration > 0.5)
        {
            accumulate_time_acc += dt;
        }
        else if (acceleration < -0.5)
        {
            accumulate_time_dec += dt;
        }
        else if (std::fabs(acceleration) < 0.2 && intent_trajectory.trajectory_in_prediction().trajectory_points(i).v() > 0.1)
        {
            accumulate_time_keep += dt;
        }
        else if (std::fabs(acceleration) < 0.2 && intent_trajectory.trajectory_in_prediction().trajectory_points(i).v() <= 0.1)
        {
            accumulate_time_stop += dt;
        }
    }
    if (accumulate_time_acc > 0.7 * predict_time && accumulate_time_acc >= accumulate_time_dec)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_ACCELERATING_STATE);
        return true;
    }
    else if (accumulate_time_dec > 0.7 * predict_time && accumulate_time_acc < accumulate_time_dec)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_SLOWDOWN_STATE);
        return true;
    }
    else if (accumulate_time_keep > 0.7 * predict_time)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_KEEP_SPEED_STATE);
        return true;
    }
    else if (accumulate_time_stop > 0.7 * predict_time)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_STOPPING_STATE);
        return true;
    }

    intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_FORWARD_STATE);

    return true;
}

/**
 * @brief Predict for vehicle
 */
bool IntentPrediction::PredictIntentForVehicle(const std::vector<legionclaw::prediction::PredictionTrajectory> &trajectories,
                                               const double &predict_time,
                                               legionclaw::interface::ObstacleIntent &intent)
{
    /*取出概率最大的轨迹*/
    double max_probility = -1;
    int max_index = -1;

    if (trajectories.size() == 0)
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_STOPPING_STATE);
        return true;
    }

    for (unsigned int ip = 0; ip < trajectories.size(); ip++)
    {
        if (trajectories.at(ip).trajectory_in_prediction().probability() > max_probility)
        {
            max_probility = trajectories.at(ip).trajectory_in_prediction().probability();
            max_index = ip;
        }
    }
    if (max_index < 0 || max_index > int(trajectories.size()) || max_probility < 0)
        return false;

    legionclaw::prediction::PredictionTrajectory intent_trajectory;
    intent_trajectory = trajectories.at(max_index);

    if (intent_trajectory.trajectory_in_prediction().trajectory_points_size() == 0)
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_STOPPING_STATE);
        return true;
    }

    /*Turn Left/Right*/
    double start_heading = intent_trajectory.trajectory_in_prediction().mutable_trajectory_points()->front().path_point().theta();
    double end_heading = intent_trajectory.trajectory_in_prediction().mutable_trajectory_points()->back().path_point().theta();
    double delta_heading = legionclaw::common::math::AngleDiff(start_heading, end_heading);
    if (delta_heading > M_PI_4) //Turn Left
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_TURN_LEFT_STATE);
        return true;
    }
    else if (delta_heading < -M_PI_4) //Turn Right
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_TURN_RIGHT_STATE);
        return true;
    }

    double acceleration = 0.0;
    double dt = 0.0;
    double accumulate_time_acc = 0.0;
    double accumulate_time_dec = 0.0;
    double accumulate_time_keep = 0.0;

    /*Change Lane Left/Right*/
    if (intent_trajectory.predict_direction() == legionclaw::prediction::PREDICT_DIR_LEFT)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_CHANGE_LANE_LEFT_STATE);
        return true;
    }
    else if (intent_trajectory.predict_direction() == legionclaw::prediction::PREDICT_DIR_RIGHT)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_CHANGE_LANE_RIGHT_STATE);
        return true;
    }
    else if (intent_trajectory.predict_direction() == legionclaw::prediction::PREDICT_DIR_STRAIGHT)
    {
        intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_FORWARD_STATE);
        return true;
    }

    for (unsigned int i = 0; i < intent_trajectory.trajectory_in_prediction().trajectory_points_size() - 1; ++i)
    {
        /*Acceleration/Deceleration*/
        //暂时不考虑连续性
        dt = intent_trajectory.trajectory_in_prediction().trajectory_points(i + 1).relative_time() -
             intent_trajectory.trajectory_in_prediction().trajectory_points(i).relative_time();
        acceleration = (intent_trajectory.trajectory_in_prediction().trajectory_points(i + 1).v() -
                        intent_trajectory.trajectory_in_prediction().trajectory_points(i).v()) /
                       dt;
        if (acceleration > 0.5)
        {
            accumulate_time_acc += dt;
        }
        else if (acceleration < -0.5)
        {
            accumulate_time_dec += dt;
        }
        else
            accumulate_time_keep += dt;
    }
    if (accumulate_time_acc >= accumulate_time_dec && accumulate_time_acc > 0.7 * predict_time)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_ACCELERATING_STATE);
        return true;
    }
    else if (accumulate_time_acc < accumulate_time_dec && accumulate_time_dec > 0.7 * predict_time)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_SLOWDOWN_STATE);
        return true;
    }
    else if (accumulate_time_keep > 0.7 * predict_time)
    {
         intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_KEEP_SPEED_STATE);
        return true;
    }

    intent.set_type(legionclaw::interface::ObstacleIntent::Type::INTENT_FORWARD_STATE);
    return true;
}
