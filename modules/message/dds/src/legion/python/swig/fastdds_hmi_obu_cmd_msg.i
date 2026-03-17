%module fastdds_hmi_obu_cmd_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HMIObuCmdPubSubTypes.h"
#include "HMIObuCmdMsgPubSubTypes.h"
#include "TimePubSubTypes.h"
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
    %template(VectorHMIObuCmd) vector<ros2_interface::msg::HMIObuCmd>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class HMIObuCmd
    {
    public:
        HMIObuCmd();
        ~HMIObuCmd();

        void code(
            int _code);
        int code() const;
        void val(
            int _val);
        int val() const;
    };

    class HMIObuCmdMsg
    {
    public:
        HMIObuCmdMsg();
        ~HMIObuCmdMsg();

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
            const std::vector<ros2_interface::msg::HMIObuCmd> &_obu_cmd_list);
        const std::vector<ros2_interface::msg::HMIObuCmd>& obu_cmd_list() const;
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



    class HMIObuCmdPubSubType
    {
    public:
        typedef HMIObuCmd type;

        HMIObuCmdPubSubType();
        ~HMIObuCmdPubSubType();
    };

    class HMIObuCmdMsgPubSubType
    {
    public:
        typedef HMIObuCmdMsg type;

        HMIObuCmdMsgPubSubType();
        ~HMIObuCmdMsgPubSubType();
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


%template(HMIObuCmdMsgDataWriter) FastddsDataWriter<ros2_interface::msg::HMIObuCmdMsgPubSubType>;
%template(HMIObuCmdMsgDataReader) FastddsDataReader<ros2_interface::msg::HMIObuCmdMsgPubSubType>;
