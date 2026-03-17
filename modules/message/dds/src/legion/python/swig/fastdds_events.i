%module fastdds_events 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimePubSubTypes.h"
#include "EventPubSubTypes.h"
#include "EventsPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef unsigned int uint64_t;
 typedef int int32_t;



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
    %template(VectorEvent) vector<ros2_interface::msg::Event>;
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

    class Events
    {
    public:
        Events();
        ~Events();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void version(
            int _version);
        int version() const;
        void events(
            const std::vector<ros2_interface::msg::Event> &_events);
        const std::vector<ros2_interface::msg::Event>& events() const;
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

    class EventsPubSubType
    {
    public:
        typedef Events type;

        EventsPubSubType();
        ~EventsPubSubType();
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


%template(EventsDataWriter) FastddsDataWriter<ros2_interface::msg::EventsPubSubType>;
%template(EventsDataReader) FastddsDataReader<ros2_interface::msg::EventsPubSubType>;
