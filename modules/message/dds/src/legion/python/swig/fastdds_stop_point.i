%module fastdds_stop_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "StopPointPubSubTypes.h"
#include "Point3DPubSubTypes.h"
%}
 typedef int int8_t;



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
    class StopPoint
    {
    public:
        StopPoint();
        ~StopPoint();

        void point(
                const ros2_interface::msg::Point3D &_point);
        const ros2_interface::msg::Point3D &point() const;
        void theta(
            double _theta);
        double theta() const;
        void type(
            int _type);
        int type() const;
        void stop_distance(
            double _stop_distance);
        double stop_distance() const;
    };

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



    class StopPointPubSubType
    {
    public:
        typedef StopPoint type;

        StopPointPubSubType();
        ~StopPointPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

}
}


%template(StopPointDataWriter) FastddsDataWriter<ros2_interface::msg::StopPointPubSubType>;
%template(StopPointDataReader) FastddsDataReader<ros2_interface::msg::StopPointPubSubType>;
