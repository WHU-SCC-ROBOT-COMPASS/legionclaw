%module fastdds_message 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "MessagePubSubTypes.h"
%}
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



    class MessagePubSubType
    {
    public:
        typedef Message type;

        MessagePubSubType();
        ~MessagePubSubType();
    };

}
}


%template(MessageDataWriter) FastddsDataWriter<ros2_interface::msg::MessagePubSubType>;
%template(MessageDataReader) FastddsDataReader<ros2_interface::msg::MessagePubSubType>;
