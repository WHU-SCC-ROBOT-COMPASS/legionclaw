%module fastdds_adc_trajectory 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "RSSInfoPubSubTypes.h"
#include "EStopPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
#include "ADCTrajectoryPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
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
    %template(VectorTrajectoryPoint) vector<ros2_interface::msg::TrajectoryPoint>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class PathPoint
    {
    public:
        PathPoint();
        ~PathPoint();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void theta(
            double _theta);
        double theta() const;
        void kappa(
            double _kappa);
        double kappa() const;
        void s(
            double _s);
        double s() const;
        void dkappa(
            double _dkappa);
        double dkappa() const;
        void ddkappa(
            double _ddkappa);
        double ddkappa() const;
        void lane_id(
            double _lane_id);
        double lane_id() const;
        void x_derivative(
            double _x_derivative);
        double x_derivative() const;
        void y_derivative(
            double _y_derivative);
        double y_derivative() const;
    };

    class RSSInfo
    {
    public:
        RSSInfo();
        ~RSSInfo();

        void is_rss_safe(
            bool _is_rss_safe);
        bool is_rss_safe() const;
        void cur_dist_lon(
            double _cur_dist_lon);
        double cur_dist_lon() const;
        void rss_safe_dist_lon(
            double _rss_safe_dist_lon);
        double rss_safe_dist_lon() const;
        void acc_lon_range_minimum(
            double _acc_lon_range_minimum);
        double acc_lon_range_minimum() const;
        void acc_lon_range_maximum(
            double _acc_lon_range_maximum);
        double acc_lon_range_maximum() const;
        void acc_lat_left_range_minimum(
            double _acc_lat_left_range_minimum);
        double acc_lat_left_range_minimum() const;
        void acc_lat_left_range_maximum(
            double _acc_lat_left_range_maximum);
        double acc_lat_left_range_maximum() const;
        void acc_lat_right_range_minimum(
            double _acc_lat_right_range_minimum);
        double acc_lat_right_range_minimum() const;
        void acc_lat_right_range_maximum(
            double _acc_lat_right_range_maximum);
        double acc_lat_right_range_maximum() const;
    };

    class EStop
    {
    public:
        EStop();
        ~EStop();

        void is_estop(
            bool _is_estop);
        bool is_estop() const;
        void reason(
           const std::string &_reason);
        std::string reason() const;
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

    class TrajectoryPoint
    {
    public:
        TrajectoryPoint();
        ~TrajectoryPoint();

        void path_point(
                const ros2_interface::msg::PathPoint &_path_point);
        const ros2_interface::msg::PathPoint &path_point() const;
        void v(
            double _v);
        double v() const;
        void a(
            double _a);
        double a() const;
        void relative_time(
            double _relative_time);
        double relative_time() const;
        void da(
            double _da);
        double da() const;
        void is_steer_valid(
            bool _is_steer_valid);
        bool is_steer_valid() const;
        void front_steer(
            double _front_steer);
        double front_steer() const;
        void rear_steer(
            double _rear_steer);
        double rear_steer() const;
        void gear(
            int _gear);
        int gear() const;
    };

    class ADCTrajectory
    {
    public:
        ADCTrajectory();
        ~ADCTrajectory();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void total_path_length(
            double _total_path_length);
        double total_path_length() const;
        void total_path_time(
            double _total_path_time);
        double total_path_time() const;
        void trajectory_points(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_trajectory_points);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& trajectory_points() const;
        void car_action(
            int _car_action);
        int car_action() const;
        void behaviour_lat_state(
            int _behaviour_lat_state);
        int behaviour_lat_state() const;
        void behaviour_lon_state(
            int _behaviour_lon_state);
        int behaviour_lon_state() const;
        void scenario(
            int _scenario);
        int scenario() const;
        void driving_mode(
            int _driving_mode);
        int driving_mode() const;
        void adc_trajectory_type(
            int _adc_trajectory_type);
        int adc_trajectory_type() const;
        void estop(
                const ros2_interface::msg::EStop &_estop);
        const ros2_interface::msg::EStop &estop() const;
        void is_replan(
            bool _is_replan);
        bool is_replan() const;
        void replan_reason(
           const std::string &_replan_reason);
        std::string replan_reason() const;
        void right_of_way_status(
            int _right_of_way_status);
        int right_of_way_status() const;
        void rss_info(
                const ros2_interface::msg::RSSInfo &_rss_info);
        const ros2_interface::msg::RSSInfo &rss_info() const;
    };



    class PathPointPubSubType
    {
    public:
        typedef PathPoint type;

        PathPointPubSubType();
        ~PathPointPubSubType();
    };

    class RSSInfoPubSubType
    {
    public:
        typedef RSSInfo type;

        RSSInfoPubSubType();
        ~RSSInfoPubSubType();
    };

    class EStopPubSubType
    {
    public:
        typedef EStop type;

        EStopPubSubType();
        ~EStopPubSubType();
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

    class TrajectoryPointPubSubType
    {
    public:
        typedef TrajectoryPoint type;

        TrajectoryPointPubSubType();
        ~TrajectoryPointPubSubType();
    };

    class ADCTrajectoryPubSubType
    {
    public:
        typedef ADCTrajectory type;

        ADCTrajectoryPubSubType();
        ~ADCTrajectoryPubSubType();
    };

}
}


%template(ADCTrajectoryDataWriter) FastddsDataWriter<ros2_interface::msg::ADCTrajectoryPubSubType>;
%template(ADCTrajectoryDataReader) FastddsDataReader<ros2_interface::msg::ADCTrajectoryPubSubType>;
