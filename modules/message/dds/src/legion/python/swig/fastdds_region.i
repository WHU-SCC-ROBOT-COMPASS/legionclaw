%module fastdds_region 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "RegionPubSubTypes.h"
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
    %template(VectorPoint3D) vector<ros2_interface::msg::Point3D>;
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

}
}


%template(RegionDataWriter) FastddsDataWriter<ros2_interface::msg::RegionPubSubType>;
%template(RegionDataReader) FastddsDataReader<ros2_interface::msg::RegionPubSubType>;
