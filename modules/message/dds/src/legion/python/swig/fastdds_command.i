%module fastdds_command 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "CommandPubSubTypes.h"
%}
%include "std_string.i"
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
    class Command
    {
    public:
        Command();
        ~Command();

        void target_module(
           const std::string &_target_module);
        std::string target_module() const;
        void target_id(
            int _target_id);
        int target_id() const;
        void target_switch(
            int _target_switch);
        int target_switch() const;
    };



    class CommandPubSubType
    {
    public:
        typedef Command type;

        CommandPubSubType();
        ~CommandPubSubType();
    };

}
}


%template(CommandDataWriter) FastddsDataWriter<ros2_interface::msg::CommandPubSubType>;
%template(CommandDataReader) FastddsDataReader<ros2_interface::msg::CommandPubSubType>;
