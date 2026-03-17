%module fastdds_event 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimePubSubTypes.h"
#include "EventPubSubTypes.h"
%}
%include "std_string.i"
 typedef unsigned int uint32_t;
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

    class Event
    {
    public:
        Event();
        ~Event();

        void timestamp(
                const ros2_interface::msg::Time &_timestamp);
        const ros2_interface::msg::Time &timestamp() const;
        void code(
            int _code);
        int code() const;
        void reason(
           const std::string &_reason);
        std::string reason() const;
    };



    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class EventPubSubType
    {
    public:
        typedef Event type;

        EventPubSubType();
        ~EventPubSubType();
    };

}
}


%template(EventDataWriter) FastddsDataWriter<ros2_interface::msg::EventPubSubType>;
%template(EventDataReader) FastddsDataReader<ros2_interface::msg::EventPubSubType>;
