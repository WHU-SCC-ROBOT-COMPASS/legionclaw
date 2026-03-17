%module fastdds_radar_obstacle_list_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "StatusPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "RadarStateErrorPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "Point2DPubSubTypes.h"
#include "RadarObstaclePubSubTypes.h"
#include "RadarStatePubSubTypes.h"
#include "RadarObstacleListMsgPubSubTypes.h"
#include "RadarStateModePubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef unsigned int uint8_t;
 typedef int int32_t;
 typedef unsigned int uint16_t;



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
    %template(VectorRadarObstacle) vector<ros2_interface::msg::RadarObstacle>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Status
    {
    public:
        Status();
        ~Status();

        void error_code(
            int _error_code);
        int error_code() const;
        void msg(
           const std::string &_msg);
        std::string msg() const;
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

    class RadarStateError
    {
    public:
        RadarStateError();
        ~RadarStateError();

        void persistent_error(
            int _persistent_error);
        int persistent_error() const;
        void temporary_error(
            int _temporary_error);
        int temporary_error() const;
        void interference_error(
            int _interference_error);
        int interference_error() const;
        void temperature_error(
            int _temperature_error);
        int temperature_error() const;
        void voltage_error(
            int _voltage_error);
        int voltage_error() const;
        void block_error(
            int _block_error);
        int block_error() const;
        void broadcast_error(
            int _broadcast_error);
        int broadcast_error() const;
        void electric_axis_error(
            int _electric_axis_error);
        int electric_axis_error() const;
        void config_error(
            int _config_error);
        int config_error() const;
        void calibration_error(
            int _calibration_error);
        int calibration_error() const;
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

    class Point2D
    {
    public:
        Point2D();
        ~Point2D();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
    };

    class RadarObstacle
    {
    public:
        RadarObstacle();
        ~RadarObstacle();

        void id(
            int _id);
        int id() const;
        void life_time(
            int _life_time);
        int life_time() const;
        void relative_position(
                const ros2_interface::msg::Point2D &_relative_position);
        const ros2_interface::msg::Point2D &relative_position() const;
        void relative_position_rms(
                const ros2_interface::msg::Point2D &_relative_position_rms);
        const ros2_interface::msg::Point2D &relative_position_rms() const;
        void relative_velocity(
                const ros2_interface::msg::Point2D &_relative_velocity);
        const ros2_interface::msg::Point2D &relative_velocity() const;
        void relative_velocity_rms(
                const ros2_interface::msg::Point2D &_relative_velocity_rms);
        const ros2_interface::msg::Point2D &relative_velocity_rms() const;
        void relative_acceleration(
                const ros2_interface::msg::Point2D &_relative_acceleration);
        const ros2_interface::msg::Point2D &relative_acceleration() const;
        void relative_acceleration_rms(
                const ros2_interface::msg::Point2D &_relative_acceleration_rms);
        const ros2_interface::msg::Point2D &relative_acceleration_rms() const;
        void rcs(
            double _rcs);
        double rcs() const;
        void snr(
            double _snr);
        double snr() const;
        void moving_status(
            int _moving_status);
        int moving_status() const;
        void width(
            double _width);
        double width() const;
        void length(
            double _length);
        double length() const;
        void height(
            double _height);
        double height() const;
        void theta(
            double _theta);
        double theta() const;
        void absolute_position(
                const ros2_interface::msg::Point2D &_absolute_position);
        const ros2_interface::msg::Point2D &absolute_position() const;
        void absolute_position_rms(
                const ros2_interface::msg::Point2D &_absolute_position_rms);
        const ros2_interface::msg::Point2D &absolute_position_rms() const;
        void absolute_velocity(
                const ros2_interface::msg::Point2D &_absolute_velocity);
        const ros2_interface::msg::Point2D &absolute_velocity() const;
        void absolute_velocity_rms(
                const ros2_interface::msg::Point2D &_absolute_velocity_rms);
        const ros2_interface::msg::Point2D &absolute_velocity_rms() const;
        void absolute_acceleration(
                const ros2_interface::msg::Point2D &_absolute_acceleration);
        const ros2_interface::msg::Point2D &absolute_acceleration() const;
        void absolute_acceleration_rms(
                const ros2_interface::msg::Point2D &_absolute_acceleration_rms);
        const ros2_interface::msg::Point2D &absolute_acceleration_rms() const;
        void orientation(
            double _orientation);
        double orientation() const;
        void orient_rms(
            double _orient_rms);
        double orient_rms() const;
        void yaw(
            double _yaw);
        double yaw() const;
        void yaw_rms(
            double _yaw_rms);
        double yaw_rms() const;
        void count(
            int _count);
        int count() const;
        void moving_frames_count(
            int _moving_frames_count);
        int moving_frames_count() const;
        void status(
                const ros2_interface::msg::Status &_status);
        const ros2_interface::msg::Status &status() const;
        void underpass_probability(
            double _underpass_probability);
        double underpass_probability() const;
        void overpass_probability(
            double _overpass_probability);
        double overpass_probability() const;
        void exist_probability(
            int _exist_probability);
        int exist_probability() const;
        void mov_property(
            int _mov_property);
        int mov_property() const;
        void track_state(
            int _track_state);
        int track_state() const;
        void track_type(
            int _track_type);
        int track_type() const;
    };

    class RadarState
    {
    public:
        RadarState();
        ~RadarState();

        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void nvm_read_status(
            int _nvm_read_status);
        int nvm_read_status() const;
        void nvm_write_status(
            int _nvm_write_status);
        int nvm_write_status() const;
        void radar_state_error(
                const ros2_interface::msg::RadarStateError &_radar_state_error);
        const ros2_interface::msg::RadarStateError &radar_state_error() const;
        void radar_state_mode(
                const ros2_interface::msg::RadarStateMode &_radar_state_mode);
        const ros2_interface::msg::RadarStateMode &radar_state_mode() const;
        void max_distance(
            int _max_distance);
        int max_distance() const;
        void sort_index(
            int _sort_index);
        int sort_index() const;
        void radar_power(
            int _radar_power);
        int radar_power() const;
        void ctl_relay(
            int _ctl_relay);
        int ctl_relay() const;
        void output_type(
            int _output_type);
        int output_type() const;
        void send_quality(
            int _send_quality);
        int send_quality() const;
        void send_extinfo(
            int _send_extinfo);
        int send_extinfo() const;
        void motion_rx_state(
            int _motion_rx_state);
        int motion_rx_state() const;
        void rcs_threshold(
            int _rcs_threshold);
        int rcs_threshold() const;
        void connector_direction(
            int _connector_direction);
        int connector_direction() const;
        void radar_position(
            int _radar_position);
        int radar_position() const;
        void hw_error(
            int _hw_error);
        int hw_error() const;
    };

    class RadarObstacleListMsg
    {
    public:
        RadarObstacleListMsg();
        ~RadarObstacleListMsg();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void radar_obstacle(
            const std::vector<ros2_interface::msg::RadarObstacle> &_radar_obstacle);
        const std::vector<ros2_interface::msg::RadarObstacle>& radar_obstacle() const;
        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void radar_state(
                const ros2_interface::msg::RadarState &_radar_state);
        const ros2_interface::msg::RadarState &radar_state() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
    };

    class RadarStateMode
    {
    public:
        RadarStateMode();
        ~RadarStateMode();

        void can0_work_mode(
            int _can0_work_mode);
        int can0_work_mode() const;
        void can1_work_mode(
            int _can1_work_mode);
        int can1_work_mode() const;
        void dual_can_mode(
            int _dual_can_mode);
        int dual_can_mode() const;
        void timming_mode(
            int _timming_mode);
        int timming_mode() const;
        void power_mode(
            int _power_mode);
        int power_mode() const;
        void performance_mode(
            int _performance_mode);
        int performance_mode() const;
    };



    class StatusPubSubType
    {
    public:
        typedef Status type;

        StatusPubSubType();
        ~StatusPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class RadarStateErrorPubSubType
    {
    public:
        typedef RadarStateError type;

        RadarStateErrorPubSubType();
        ~RadarStateErrorPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

    class RadarObstaclePubSubType
    {
    public:
        typedef RadarObstacle type;

        RadarObstaclePubSubType();
        ~RadarObstaclePubSubType();
    };

    class RadarStatePubSubType
    {
    public:
        typedef RadarState type;

        RadarStatePubSubType();
        ~RadarStatePubSubType();
    };

    class RadarObstacleListMsgPubSubType
    {
    public:
        typedef RadarObstacleListMsg type;

        RadarObstacleListMsgPubSubType();
        ~RadarObstacleListMsgPubSubType();
    };

    class RadarStateModePubSubType
    {
    public:
        typedef RadarStateMode type;

        RadarStateModePubSubType();
        ~RadarStateModePubSubType();
    };

}
}


%template(RadarObstacleListMsgDataWriter) FastddsDataWriter<ros2_interface::msg::RadarObstacleListMsgPubSubType>;
%template(RadarObstacleListMsgDataReader) FastddsDataReader<ros2_interface::msg::RadarObstacleListMsgPubSubType>;
