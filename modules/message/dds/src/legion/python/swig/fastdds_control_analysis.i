%module fastdds_control_analysis 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ControlAnalysisPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int64_t;
 typedef unsigned int uint32_t;



template <class CustomTopicDataType>
class FastddsDataWriter
{
public:
    typedef typename CustomTopicDataType::type type;

    FastddsDataWriter(
        uint32_t did,
        const std::string &topic_name);
    ~FastddsDataWriter();

    bool write_sample(
        type &msg);
};

template <class CustomTopicDataType>
class FastddsDataReader
{
public:
    typedef typename CustomTopicDataType::type type;

    FastddsDataReader(
        uint32_t did,
        const std::string &topic_name);
    ~FastddsDataReader();

    bool wait_for_sample(
        uint32_t seconds);
    bool take_sample(
        type &msg);
};
namespace std {
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class ControlAnalysis
    {
    public:
        ControlAnalysis();
        ~ControlAnalysis();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void driving_mode(
            int _driving_mode);
        int driving_mode() const;
        void driving_mode_fd(
            int _driving_mode_fd);
        int driving_mode_fd() const;
        void gear_location_fd(
            int _gear_location_fd);
        int gear_location_fd() const;
        void gear_location_cmd(
            int _gear_location_cmd);
        int gear_location_cmd() const;
        void epb_level_fd(
            int _epb_level_fd);
        int epb_level_fd() const;
        void epb_level_cmd(
            int _epb_level_cmd);
        int epb_level_cmd() const;
        void speed_mps(
            double _speed_mps);
        double speed_mps() const;
        void speed_reference(
            double _speed_reference);
        double speed_reference() const;
        void accel_value_fd(
            double _accel_value_fd);
        double accel_value_fd() const;
        void accel_value_cmd(
            double _accel_value_cmd);
        double accel_value_cmd() const;
        void brake_value_fd(
            double _brake_value_fd);
        double brake_value_fd() const;
        void brake_value_cmd(
            double _brake_value_cmd);
        double brake_value_cmd() const;
        void path_remain(
            double _path_remain);
        double path_remain() const;
        void has_stop_point(
            bool _has_stop_point);
        bool has_stop_point() const;
        void is_full_stop(
            bool _is_full_stop);
        bool is_full_stop() const;
        void is_stopped(
            bool _is_stopped);
        bool is_stopped() const;
        void lon_acc_jerk(
            double _lon_acc_jerk);
        double lon_acc_jerk() const;
        void acceleration_cmd(
            double _acceleration_cmd);
        double acceleration_cmd() const;
        void acceleration_cmd_closeloop(
            double _acceleration_cmd_closeloop);
        double acceleration_cmd_closeloop() const;
        void preview_acceleration_reference(
            double _preview_acceleration_reference);
        double preview_acceleration_reference() const;
        void slope_offset_compensation(
            double _slope_offset_compensation);
        double slope_offset_compensation() const;
        void turning_offset_compensation(
            double _turning_offset_compensation);
        double turning_offset_compensation() const;
        void speed_error_limited(
            double _speed_error_limited);
        double speed_error_limited() const;
        void speed_error(
            double _speed_error);
        double speed_error() const;
        void speed_offset(
            double _speed_offset);
        double speed_offset() const;
        void station_error_limited(
            double _station_error_limited);
        double station_error_limited() const;
        void station_error(
            double _station_error);
        double station_error() const;
        void lon_target_point_s(
            double _lon_target_point_s);
        double lon_target_point_s() const;
        void lon_calculate_time(
            int _lon_calculate_time);
        int lon_calculate_time() const;
        void lon_calculate_time_max(
            int _lon_calculate_time_max);
        int lon_calculate_time_max() const;
        void ref_curvature(
            double _ref_curvature);
        double ref_curvature() const;
        void ref_heading(
            double _ref_heading);
        double ref_heading() const;
        void current_heading(
            double _current_heading);
        double current_heading() const;
        void heading_error(
            double _heading_error);
        double heading_error() const;
        void heading_error_rate(
            double _heading_error_rate);
        double heading_error_rate() const;
        void lateral_error(
            double _lateral_error);
        double lateral_error() const;
        void lateral_error_rate(
            double _lateral_error_rate);
        double lateral_error_rate() const;
        void lon_error(
            double _lon_error);
        double lon_error() const;
        void front_steering_value_fd(
            double _front_steering_value_fd);
        double front_steering_value_fd() const;
        void front_steering_target(
            double _front_steering_target);
        double front_steering_target() const;
        void front_steering_rate(
            double _front_steering_rate);
        double front_steering_rate() const;
        void front_steer_angle_feedforward(
            double _front_steer_angle_feedforward);
        double front_steer_angle_feedforward() const;
        void front_steer_angle_feedback(
            double _front_steer_angle_feedback);
        double front_steer_angle_feedback() const;
        void front_steer_angle_lateral_contribution(
            double _front_steer_angle_lateral_contribution);
        double front_steer_angle_lateral_contribution() const;
        void front_steer_angle_lateral_rate_contribution(
            double _front_steer_angle_lateral_rate_contribution);
        double front_steer_angle_lateral_rate_contribution() const;
        void front_steer_angle_heading_contribution(
            double _front_steer_angle_heading_contribution);
        double front_steer_angle_heading_contribution() const;
        void front_steer_angle_heading_rate_contribution(
            double _front_steer_angle_heading_rate_contribution);
        double front_steer_angle_heading_rate_contribution() const;
        void rear_steering_value_fd(
            double _rear_steering_value_fd);
        double rear_steering_value_fd() const;
        void rear_steering_target(
            double _rear_steering_target);
        double rear_steering_target() const;
        void rear_steering_rate(
            double _rear_steering_rate);
        double rear_steering_rate() const;
        void rear_steer_angle_feedforward(
            double _rear_steer_angle_feedforward);
        double rear_steer_angle_feedforward() const;
        void rear_steer_angle_feedback(
            double _rear_steer_angle_feedback);
        double rear_steer_angle_feedback() const;
        void rear_steer_angle_lateral_contribution(
            double _rear_steer_angle_lateral_contribution);
        double rear_steer_angle_lateral_contribution() const;
        void rear_steer_angle_lateral_rate_contribution(
            double _rear_steer_angle_lateral_rate_contribution);
        double rear_steer_angle_lateral_rate_contribution() const;
        void rear_steer_angle_heading_contribution(
            double _rear_steer_angle_heading_contribution);
        double rear_steer_angle_heading_contribution() const;
        void rear_steer_angle_heading_rate_contribution(
            double _rear_steer_angle_heading_rate_contribution);
        double rear_steer_angle_heading_rate_contribution() const;
        void matrix_k_00(
            double _matrix_k_00);
        double matrix_k_00() const;
        void matrix_k_01(
            double _matrix_k_01);
        double matrix_k_01() const;
        void matrix_k_02(
            double _matrix_k_02);
        double matrix_k_02() const;
        void matrix_k_03(
            double _matrix_k_03);
        double matrix_k_03() const;
        void matrix_k_10(
            double _matrix_k_10);
        double matrix_k_10() const;
        void matrix_k_11(
            double _matrix_k_11);
        double matrix_k_11() const;
        void matrix_k_12(
            double _matrix_k_12);
        double matrix_k_12() const;
        void matrix_k_13(
            double _matrix_k_13);
        double matrix_k_13() const;
        void matrix_state_0(
            double _matrix_state_0);
        double matrix_state_0() const;
        void matrix_state_1(
            double _matrix_state_1);
        double matrix_state_1() const;
        void matrix_state_2(
            double _matrix_state_2);
        double matrix_state_2() const;
        void matrix_state_3(
            double _matrix_state_3);
        double matrix_state_3() const;
        void matrix_q_updated_0(
            double _matrix_q_updated_0);
        double matrix_q_updated_0() const;
        void matrix_q_updated_1(
            double _matrix_q_updated_1);
        double matrix_q_updated_1() const;
        void matrix_q_updated_2(
            double _matrix_q_updated_2);
        double matrix_q_updated_2() const;
        void matrix_q_updated_3(
            double _matrix_q_updated_3);
        double matrix_q_updated_3() const;
        void current_x(
            double _current_x);
        double current_x() const;
        void current_y(
            double _current_y);
        double current_y() const;
        void target_point_x(
            double _target_point_x);
        double target_point_x() const;
        void target_point_y(
            double _target_point_y);
        double target_point_y() const;
        void lat_target_point_s(
            double _lat_target_point_s);
        double lat_target_point_s() const;
        void lqr_calculate_time(
            int _lqr_calculate_time);
        int lqr_calculate_time() const;
        void lqr_calculate_time_max(
            int _lqr_calculate_time_max);
        int lqr_calculate_time_max() const;
    };

    class Time
    {
    public:
        Time();
        ~Time();

        void sec(
            int _sec);
        int sec() const;
        void nsec(
            int _nsec);
        int nsec() const;
    };

    class Header
    {
    public:
        Header();
        ~Header();

        void seq(
            int _seq);
        int seq() const;
        void stamp(
                const ros2_interface::msg::Time &_stamp);
        const ros2_interface::msg::Time &stamp() const;
        void frame_id(
           const std::string &_frame_id);
        std::string frame_id() const;
    };



    class ControlAnalysisPubSubType
    {
    public:
        typedef ControlAnalysis type;

        ControlAnalysisPubSubType();
        ~ControlAnalysisPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

}
}


%template(ControlAnalysisDataWriter) FastddsDataWriter<ros2_interface::msg::ControlAnalysisPubSubType>;
%template(ControlAnalysisDataReader) FastddsDataReader<ros2_interface::msg::ControlAnalysisPubSubType>;
