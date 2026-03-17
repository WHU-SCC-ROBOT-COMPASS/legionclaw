%module fastdds_point_basic 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointBasicPubSubTypes.h"
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
    class PointBasic
    {
    public:
        PointBasic();
        ~PointBasic();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void theta(
            double _theta);
        double theta() const;
    };



    class PointBasicPubSubType
    {
    public:
        typedef PointBasic type;

        PointBasicPubSubType();
        ~PointBasicPubSubType();
    };

}
}


%template(PointBasicDataWriter) FastddsDataWriter<ros2_interface::msg::PointBasicPubSubType>;
%template(PointBasicDataReader) FastddsDataReader<ros2_interface::msg::PointBasicPubSubType>;
