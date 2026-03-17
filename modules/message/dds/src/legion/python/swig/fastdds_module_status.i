%module fastdds_module_status 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "MessagePubSubTypes.h"
#include "ModuleStatusPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
 typedef int int8_t;
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
    %template(VectorMessage) vector<ros2_interface::msg::Message>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Message
    {
    public:
        Message();
        ~Message();

        void code(
            int _code);
        int code() const;
        void value(
            int _value);
        int value() const;
    };

    class ModuleStatus
    {
    public:
        ModuleStatus();
        ~ModuleStatus();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void name(
           const std::string &_name);
        std::string name() const;
        void self_id(
            int _self_id);
        int self_id() const;
        void status(
            int _status);
        int status() const;
        void messages(
            const std::vector<ros2_interface::msg::Message> &_messages);
        const std::vector<ros2_interface::msg::Message>& messages() const;
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



    class MessagePubSubType
    {
    public:
        typedef Message type;

        MessagePubSubType();
        ~MessagePubSubType();
    };

    class ModuleStatusPubSubType
    {
    public:
        typedef ModuleStatus type;

        ModuleStatusPubSubType();
        ~ModuleStatusPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
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


%template(ModuleStatusDataWriter) FastddsDataWriter<ros2_interface::msg::ModuleStatusPubSubType>;
%template(ModuleStatusDataReader) FastddsDataReader<ros2_interface::msg::ModuleStatusPubSubType>;
