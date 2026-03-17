%module fastdds_speed_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SpeedPointPubSubTypes.h"
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
    class SpeedPoint
    {
    public:
        SpeedPoint();
        ~SpeedPoint();

        void s(
            double _s);
        double s() const;
        void t(
            double _t);
        double t() const;
        void v(
            double _v);
        double v() const;
        void a(
            double _a);
        double a() const;
        void da(
            double _da);
        double da() const;
    };



    class SpeedPointPubSubType
    {
    public:
        typedef SpeedPoint type;

        SpeedPointPubSubType();
        ~SpeedPointPubSubType();
    };

}
}


%template(SpeedPointDataWriter) FastddsDataWriter<ros2_interface::msg::SpeedPointPubSubType>;
%template(SpeedPointDataReader) FastddsDataReader<ros2_interface::msg::SpeedPointPubSubType>;
