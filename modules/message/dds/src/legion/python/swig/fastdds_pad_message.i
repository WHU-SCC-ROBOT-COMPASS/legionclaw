%module fastdds_pad_message 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PadMessagePubSubTypes.h"
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
    class PadMessage
    {
    public:
        PadMessage();
        ~PadMessage();

        void cmd(
           const std::string &_cmd);
        std::string cmd() const;
    };



    class PadMessagePubSubType
    {
    public:
        typedef PadMessage type;

        PadMessagePubSubType();
        ~PadMessagePubSubType();
    };

}
}


%template(PadMessageDataWriter) FastddsDataWriter<ros2_interface::msg::PadMessagePubSubType>;
%template(PadMessageDataReader) FastddsDataReader<ros2_interface::msg::PadMessagePubSubType>;
