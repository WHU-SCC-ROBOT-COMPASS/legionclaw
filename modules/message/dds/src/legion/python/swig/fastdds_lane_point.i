%module fastdds_lane_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "LanePointPubSubTypes.h"
%}



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

    class LanePointPubSubType
    {
    public:
        typedef LanePoint type;

        LanePointPubSubType();
        ~LanePointPubSubType();
    };

}
}


%template(LanePointDataWriter) FastddsDataWriter<ros2_interface::msg::LanePointPubSubType>;
%template(LanePointDataReader) FastddsDataReader<ros2_interface::msg::LanePointPubSubType>;
