%module fastdds_road_mark_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RoadMarkPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "RoadMarkListPubSubTypes.h"
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
    %template(VectorRoadMark) vector<ros2_interface::msg::RoadMark>;
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

    class RoadMarkList
    {
    public:
        RoadMarkList();
        ~RoadMarkList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void roadmarks(
            const std::vector<ros2_interface::msg::RoadMark> &_roadmarks);
        const std::vector<ros2_interface::msg::RoadMark>& roadmarks() const;
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



    class RoadMarkPubSubType
    {
    public:
        typedef RoadMark type;

        RoadMarkPubSubType();
        ~RoadMarkPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class RoadMarkListPubSubType
    {
    public:
        typedef RoadMarkList type;

        RoadMarkListPubSubType();
        ~RoadMarkListPubSubType();
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


%template(RoadMarkListDataWriter) FastddsDataWriter<ros2_interface::msg::RoadMarkListPubSubType>;
%template(RoadMarkListDataReader) FastddsDataReader<ros2_interface::msg::RoadMarkListPubSubType>;
