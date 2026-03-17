%module fastdds_point_xyzirt 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointXYZIRTPubSubTypes.h"
%}
 typedef unsigned int uint32_t;
 typedef unsigned int uint8_t;
 typedef unsigned int uint64_t;



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
    class PointXYZIRT
    {
    public:
        PointXYZIRT();
        ~PointXYZIRT();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void intensity(
            int _intensity);
        int intensity() const;
        void ring_id(
            int _ring_id);
        int ring_id() const;
        void timestamp(
            int _timestamp);
        int timestamp() const;
    };



    class PointXYZIRTPubSubType
    {
    public:
        typedef PointXYZIRT type;

        PointXYZIRTPubSubType();
        ~PointXYZIRTPubSubType();
    };

}
}


%template(PointXYZIRTDataWriter) FastddsDataWriter<ros2_interface::msg::PointXYZIRTPubSubType>;
%template(PointXYZIRTDataReader) FastddsDataReader<ros2_interface::msg::PointXYZIRTPubSubType>;
