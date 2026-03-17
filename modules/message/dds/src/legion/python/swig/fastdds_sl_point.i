%module fastdds_sl_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SLPointPubSubTypes.h"
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



    class SLPointPubSubType
    {
    public:
        typedef SLPoint type;

        SLPointPubSubType();
        ~SLPointPubSubType();
    };

}
}


%template(SLPointDataWriter) FastddsDataWriter<ros2_interface::msg::SLPointPubSubType>;
%template(SLPointDataReader) FastddsDataReader<ros2_interface::msg::SLPointPubSubType>;
