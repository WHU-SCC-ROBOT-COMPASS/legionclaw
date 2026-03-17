%module fastdds_grid 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SLPointPubSubTypes.h"
#include "GridPubSubTypes.h"
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
    class SLPoint
    {
    public:
        SLPoint();
        ~SLPoint();

        void s(
            double _s);
        double s() const;
        void l(
            double _l);
        double l() const;
    };

    class Grid
    {
    public:
        Grid();
        ~Grid();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void sl_point(
                const ros2_interface::msg::SLPoint &_sl_point);
        const ros2_interface::msg::SLPoint &sl_point() const;
        void yaw(
            double _yaw);
        double yaw() const;
        void potential(
            double _potential);
        double potential() const;
        void region_id(
            int _region_id);
        int region_id() const;
    };



    class SLPointPubSubType
    {
    public:
        typedef SLPoint type;

        SLPointPubSubType();
        ~SLPointPubSubType();
    };

    class GridPubSubType
    {
    public:
        typedef Grid type;

        GridPubSubType();
        ~GridPubSubType();
    };

}
}


%template(GridDataWriter) FastddsDataWriter<ros2_interface::msg::GridPubSubType>;
%template(GridDataReader) FastddsDataReader<ros2_interface::msg::GridPubSubType>;
