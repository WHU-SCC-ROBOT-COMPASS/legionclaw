%module fastdds_road_mark 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RoadMarkPubSubTypes.h"
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
    class RoadMark
    {
    public:
        RoadMark();
        ~RoadMark();

        void longitude_dist(
            double _longitude_dist);
        double longitude_dist() const;
        void lateral_dist(
            double _lateral_dist);
        double lateral_dist() const;
        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void type(
            int _type);
        int type() const;
    };



    class RoadMarkPubSubType
    {
    public:
        typedef RoadMark type;

        RoadMarkPubSubType();
        ~RoadMarkPubSubType();
    };

}
}


%template(RoadMarkDataWriter) FastddsDataWriter<ros2_interface::msg::RoadMarkPubSubType>;
%template(RoadMarkDataReader) FastddsDataReader<ros2_interface::msg::RoadMarkPubSubType>;
