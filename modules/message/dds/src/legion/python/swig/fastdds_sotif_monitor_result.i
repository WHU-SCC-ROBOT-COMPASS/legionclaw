%module fastdds_sotif_monitor_result 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "RegionPubSubTypes.h"
#include "SotifMonitorResultPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "SLPointPubSubTypes.h"
#include "GridPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
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
    %template(VectorRegion) vector<ros2_interface::msg::Region>;
    %template(VectorGrid) vector<ros2_interface::msg::Grid>;
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

    class Region
    {
    public:
        Region();
        ~Region();

        void name_region(
            int _name_region);
        int name_region() const;
        void score(
            double _score);
        double score() const;
        void rank_risk(
            int _rank_risk);
        int rank_risk() const;
        void region_polygon(
            const std::vector<ros2_interface::msg::Point3D> &_region_polygon);
        const std::vector<ros2_interface::msg::Point3D>& region_polygon() const;
    };

    class SotifMonitorResult
    {
    public:
        SotifMonitorResult();
        ~SotifMonitorResult();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void region_value(
            const std::vector<ros2_interface::msg::Region> &_region_value);
        const std::vector<ros2_interface::msg::Region>& region_value() const;
        void grid_map(
            const std::vector<ros2_interface::msg::Grid> &_grid_map);
        const std::vector<ros2_interface::msg::Grid>& grid_map() const;
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



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class RegionPubSubType
    {
    public:
        typedef Region type;

        RegionPubSubType();
        ~RegionPubSubType();
    };

    class SotifMonitorResultPubSubType
    {
    public:
        typedef SotifMonitorResult type;

        SotifMonitorResultPubSubType();
        ~SotifMonitorResultPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
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

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

}
}


%template(SotifMonitorResultDataWriter) FastddsDataWriter<ros2_interface::msg::SotifMonitorResultPubSubType>;
%template(SotifMonitorResultDataReader) FastddsDataReader<ros2_interface::msg::SotifMonitorResultPubSubType>;
