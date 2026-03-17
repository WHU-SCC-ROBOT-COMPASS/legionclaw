%module fastdds_traffic_events 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "TrafficEventsPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "LimitSpeedInfoPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "JunctionInfoPubSubTypes.h"
#include "RouteFusionInfoPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef int int32_t;



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
    class Point3D
    {
    public:
        Point3D();
        ~Point3D();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
    };

    class TrafficEvents
    {
    public:
        TrafficEvents();
        ~TrafficEvents();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void route_fusion_info(
                const ros2_interface::msg::RouteFusionInfo &_route_fusion_info);
        const ros2_interface::msg::RouteFusionInfo &route_fusion_info() const;
        void junction_info(
                const ros2_interface::msg::JunctionInfo &_junction_info);
        const ros2_interface::msg::JunctionInfo &junction_info() const;
        void limit_speed_info(
                const ros2_interface::msg::LimitSpeedInfo &_limit_speed_info);
        const ros2_interface::msg::LimitSpeedInfo &limit_speed_info() const;
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

    class LimitSpeedInfo
    {
    public:
        LimitSpeedInfo();
        ~LimitSpeedInfo();

        void limitspeed_valid_flag(
            int _limitspeed_valid_flag);
        int limitspeed_valid_flag() const;
        void limit_speed(
            double _limit_speed);
        double limit_speed() const;
        void limit_distance(
            double _limit_distance);
        double limit_distance() const;
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

    class JunctionInfo
    {
    public:
        JunctionInfo();
        ~JunctionInfo();

        void id(
            int _id);
        int id() const;
        void light_flag(
            int _light_flag);
        int light_flag() const;
        void light_color(
            int _light_color);
        int light_color() const;
        void light_remain_time(
            double _light_remain_time);
        double light_remain_time() const;
        void distance_to_stop(
            double _distance_to_stop);
        double distance_to_stop() const;
        void direction_flag(
            int _direction_flag);
        int direction_flag() const;
        void direction(
            int _direction);
        int direction() const;
        void distance_to_junction(
            double _distance_to_junction);
        double distance_to_junction() const;
        void stop_line(
            const std::vector<ros2_interface::msg::Point3D> &_stop_line);
        const std::vector<ros2_interface::msg::Point3D>& stop_line() const;
    };

    class RouteFusionInfo
    {
    public:
        RouteFusionInfo();
        ~RouteFusionInfo();

        void fusion_flag(
            int _fusion_flag);
        int fusion_flag() const;
        void fusion_reason(
           const std::string &_fusion_reason);
        std::string fusion_reason() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class TrafficEventsPubSubType
    {
    public:
        typedef TrafficEvents type;

        TrafficEventsPubSubType();
        ~TrafficEventsPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class LimitSpeedInfoPubSubType
    {
    public:
        typedef LimitSpeedInfo type;

        LimitSpeedInfoPubSubType();
        ~LimitSpeedInfoPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class JunctionInfoPubSubType
    {
    public:
        typedef JunctionInfo type;

        JunctionInfoPubSubType();
        ~JunctionInfoPubSubType();
    };

    class RouteFusionInfoPubSubType
    {
    public:
        typedef RouteFusionInfo type;

        RouteFusionInfoPubSubType();
        ~RouteFusionInfoPubSubType();
    };

}
}


%template(TrafficEventsDataWriter) FastddsDataWriter<ros2_interface::msg::TrafficEventsPubSubType>;
%template(TrafficEventsDataReader) FastddsDataReader<ros2_interface::msg::TrafficEventsPubSubType>;
