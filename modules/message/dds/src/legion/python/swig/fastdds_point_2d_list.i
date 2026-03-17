%module fastdds_point_2d_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point2dListPubSubTypes.h"
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
    class Point2dList
    {
    public:
        Point2dList();
        ~Point2dList();

        void point2d_list(
            const std::vector<ros2_interface::msg::Point2D> &_point2d_list);
        const std::vector<ros2_interface::msg::Point2D>& point2d_list() const;
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



    class Point2dListPubSubType
    {
    public:
        typedef Point2dList type;

        Point2dListPubSubType();
        ~Point2dListPubSubType();
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


%template(Point2dListDataWriter) FastddsDataWriter<ros2_interface::msg::Point2dListPubSubType>;
%template(Point2dListDataReader) FastddsDataReader<ros2_interface::msg::Point2dListPubSubType>;
