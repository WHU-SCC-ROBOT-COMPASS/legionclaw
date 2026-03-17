%module fastdds_command_respond 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimePubSubTypes.h"
#include "CommandRespondPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef unsigned int uint32_t;
 typedef int int8_t;
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

    class CommandRespond
    {
    public:
        CommandRespond();
        ~CommandRespond();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void name(
           const std::string &_name);
        std::string name() const;
        void self_id(
            int _self_id);
        int self_id() const;
        void respond_id(
            int _respond_id);
        int respond_id() const;
        void status(
            int _status);
        int status() const;
        void fail_code(
            int _fail_code);
        int fail_code() const;
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

    class CommandRespondPubSubType
    {
    public:
        typedef CommandRespond type;

        CommandRespondPubSubType();
        ~CommandRespondPubSubType();
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


%template(CommandRespondDataWriter) FastddsDataWriter<ros2_interface::msg::CommandRespondPubSubType>;
%template(CommandRespondDataReader) FastddsDataReader<ros2_interface::msg::CommandRespondPubSubType>;
