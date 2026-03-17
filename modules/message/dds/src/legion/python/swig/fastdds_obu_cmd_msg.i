%module fastdds_obu_cmd_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ObuCmdPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "ObuCmdMsgPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
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
    %template(VectorObuCmd) vector<ros2_interface::msg::ObuCmd>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class ObuCmd
    {
    public:
        ObuCmd();
        ~ObuCmd();

        void code(
            int _code);
        int code() const;
        void val(
            int _val);
        int val() const;
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

    class ObuCmdMsg
    {
    public:
        ObuCmdMsg();
        ~ObuCmdMsg();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void id(
            int _id);
        int id() const;
        void name(
           const std::string &_name);
        std::string name() const;
        void obu_cmd_list(
            const std::vector<ros2_interface::msg::ObuCmd> &_obu_cmd_list);
        const std::vector<ros2_interface::msg::ObuCmd>& obu_cmd_list() const;
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



    class ObuCmdPubSubType
    {
    public:
        typedef ObuCmd type;

        ObuCmdPubSubType();
        ~ObuCmdPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class ObuCmdMsgPubSubType
    {
    public:
        typedef ObuCmdMsg type;

        ObuCmdMsgPubSubType();
        ~ObuCmdMsgPubSubType();
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


%template(ObuCmdMsgDataWriter) FastddsDataWriter<ros2_interface::msg::ObuCmdMsgPubSubType>;
%template(ObuCmdMsgDataReader) FastddsDataReader<ros2_interface::msg::ObuCmdMsgPubSubType>;
