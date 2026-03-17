%module fastdds_alarm_message 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimePubSubTypes.h"
#include "AlarmMessagePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef unsigned int uint32_t;
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

    class AlarmMessage
    {
    public:
        AlarmMessage();
        ~AlarmMessage();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void alarm_proc(
           const std::string &_alarm_proc);
        std::string alarm_proc() const;
        void alarm_level(
            int _alarm_level);
        int alarm_level() const;
        void alarm_type(
            int _alarm_type);
        int alarm_type() const;
        void alarm_id(
            int _alarm_id);
        int alarm_id() const;
        void alarm_display(
            int _alarm_display);
        int alarm_display() const;
        void alarm_data(
           const std::string &_alarm_data);
        std::string alarm_data() const;
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

    class AlarmMessagePubSubType
    {
    public:
        typedef AlarmMessage type;

        AlarmMessagePubSubType();
        ~AlarmMessagePubSubType();
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


%template(AlarmMessageDataWriter) FastddsDataWriter<ros2_interface::msg::AlarmMessagePubSubType>;
%template(AlarmMessageDataReader) FastddsDataReader<ros2_interface::msg::AlarmMessagePubSubType>;
