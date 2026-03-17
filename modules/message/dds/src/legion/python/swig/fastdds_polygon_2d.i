%module fastdds_polygon_2d 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Polygon2DPubSubTypes.h"
#include "Point2DPubSubTypes.h"
%}
%include "std_vector.i"



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
    %template(VectorPoint2D) vector<ros2_interface::msg::Point2D>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Polygon2D
    {
    public:
        Polygon2D();
        ~Polygon2D();

        void coordinate_system(
            int _coordinate_system);
        int coordinate_system() const;
        void points(
            const std::vector<ros2_interface::msg::Point2D> &_points);
        const std::vector<ros2_interface::msg::Point2D>& points() const;
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



    class Polygon2DPubSubType
    {
    public:
        typedef Polygon2D type;

        Polygon2DPubSubType();
        ~Polygon2DPubSubType();
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


%template(Polygon2DDataWriter) FastddsDataWriter<ros2_interface::msg::Polygon2DPubSubType>;
%template(Polygon2DDataReader) FastddsDataReader<ros2_interface::msg::Polygon2DPubSubType>;
