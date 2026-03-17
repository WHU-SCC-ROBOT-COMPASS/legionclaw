%module fastdds_time_consume 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimeConsumePubSubTypes.h"
%}
%include "std_string.i"



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
    class TimeConsume
    {
    public:
        TimeConsume();
        ~TimeConsume();

        void name(
           const std::string &_name);
        std::string name() const;
        void time_consume(
            double _time_consume);
        double time_consume() const;
    };



    class TimeConsumePubSubType
    {
    public:
        typedef TimeConsume type;

        TimeConsumePubSubType();
        ~TimeConsumePubSubType();
    };

}
}


%template(TimeConsumeDataWriter) FastddsDataWriter<ros2_interface::msg::TimeConsumePubSubType>;
%template(TimeConsumeDataReader) FastddsDataReader<ros2_interface::msg::TimeConsumePubSubType>;
