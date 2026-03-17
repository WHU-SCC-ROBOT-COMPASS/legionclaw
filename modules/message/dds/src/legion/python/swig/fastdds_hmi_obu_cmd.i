%module fastdds_hmi_obu_cmd 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HMIObuCmdPubSubTypes.h"
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



    class HMIObuCmdPubSubType
    {
    public:
        typedef HMIObuCmd type;

        HMIObuCmdPubSubType();
        ~HMIObuCmdPubSubType();
    };

}
}


%template(HMIObuCmdDataWriter) FastddsDataWriter<ros2_interface::msg::HMIObuCmdPubSubType>;
%template(HMIObuCmdDataReader) FastddsDataReader<ros2_interface::msg::HMIObuCmdPubSubType>;
