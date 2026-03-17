%module fastdds_routing_response 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "LaneInfoPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "RoutingResponsePubSubTypes.h"
#include "LanePointPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int8_t;
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
    %template(VectorLaneInfo) vector<ros2_interface::msg::LaneInfo>;
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

    class LaneInfo
    {
    public:
        LaneInfo();
        ~LaneInfo();

        void priority(
            int _priority);
        int priority() const;
        void global_id(
            int _global_id);
        int global_id() const;
        void predecessor_id(
            int _predecessor_id);
        int predecessor_id() const;
        void successor_id(
            int _successor_id);
        int successor_id() const;
        void left_neighbor_id(
            int _left_neighbor_id);
        int left_neighbor_id() const;
        void right_neighbor_id(
            int _right_neighbor_id);
        int right_neighbor_id() const;
        void type(
            int _type);
        int type() const;
        void lane_points(
            const std::vector<ros2_interface::msg::LanePoint> &_lane_points);
        const std::vector<ros2_interface::msg::LanePoint>& lane_points() const;
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

    class RoutingResponse
    {
    public:
        RoutingResponse();
        ~RoutingResponse();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void plan_status(
            int _plan_status);
        int plan_status() const;
        void replan_flag(
            int _replan_flag);
        int replan_flag() const;
        void route_reason(
            int _route_reason);
        int route_reason() const;
        void lane_list(
            const std::vector<ros2_interface::msg::LaneInfo> &_lane_list);
        const std::vector<ros2_interface::msg::LaneInfo>& lane_list() const;
    };

    class LanePoint
    {
    public:
        LanePoint();
        ~LanePoint();

        void point(
                const ros2_interface::msg::Point3D &_point);
        const ros2_interface::msg::Point3D &point() const;
        void theta(
            double _theta);
        double theta() const;
        void mileage(
            double _mileage);
        double mileage() const;
        void limit_speed(
            double _limit_speed);
        double limit_speed() const;
        void left_road_width(
            double _left_road_width);
        double left_road_width() const;
        void right_road_width(
            double _right_road_width);
        double right_road_width() const;
        void left_line_type(
            int _left_line_type);
        int left_line_type() const;
        void right_line_type(
            int _right_line_type);
        int right_line_type() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class LaneInfoPubSubType
    {
    public:
        typedef LaneInfo type;

        LaneInfoPubSubType();
        ~LaneInfoPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class RoutingResponsePubSubType
    {
    public:
        typedef RoutingResponse type;

        RoutingResponsePubSubType();
        ~RoutingResponsePubSubType();
    };

    class LanePointPubSubType
    {
    public:
        typedef LanePoint type;

        LanePointPubSubType();
        ~LanePointPubSubType();
    };

}
}


%template(RoutingResponseDataWriter) FastddsDataWriter<ros2_interface::msg::RoutingResponsePubSubType>;
%template(RoutingResponseDataReader) FastddsDataReader<ros2_interface::msg::RoutingResponsePubSubType>;
