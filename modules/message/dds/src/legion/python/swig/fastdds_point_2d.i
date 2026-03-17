%module fastdds_point_2d 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point2DPubSubTypes.h"
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



    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

}
}


%template(Point2DDataWriter) FastddsDataWriter<ros2_interface::msg::Point2DPubSubType>;
%template(Point2DDataReader) FastddsDataReader<ros2_interface::msg::Point2DPubSubType>;
