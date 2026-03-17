%module fastdds_radar_obstacle 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RadarObstaclePubSubTypes.h"
#include "StatusPubSubTypes.h"
#include "Point2DPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int32_t;
 typedef unsigned int uint8_t;



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



    class RadarObstaclePubSubType
    {
    public:
        typedef RadarObstacle type;

        RadarObstaclePubSubType();
        ~RadarObstaclePubSubType();
    };

    class StatusPubSubType
    {
    public:
        typedef Status type;

        StatusPubSubType();
        ~StatusPubSubType();
    };

    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

}
}


%template(RadarObstacleDataWriter) FastddsDataWriter<ros2_interface::msg::RadarObstaclePubSubType>;
%template(RadarObstacleDataReader) FastddsDataReader<ros2_interface::msg::RadarObstaclePubSubType>;
