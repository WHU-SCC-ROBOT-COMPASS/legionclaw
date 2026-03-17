%module fastdds_warning_command 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "WarningCommandPubSubTypes.h"
%}



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
    class WarningCommand
    {
    public:
        WarningCommand();
        ~WarningCommand();

        void sound_alarm(
            int _sound_alarm);
        int sound_alarm() const;
        void light_alarm(
            int _light_alarm);
        int light_alarm() const;
        void media_alarm(
            int _media_alarm);
        int media_alarm() const;
        void motion_alarm(
            int _motion_alarm);
        int motion_alarm() const;
    };



    class WarningCommandPubSubType
    {
    public:
        typedef WarningCommand type;

        WarningCommandPubSubType();
        ~WarningCommandPubSubType();
    };

}
}


%template(WarningCommandDataWriter) FastddsDataWriter<ros2_interface::msg::WarningCommandPubSubType>;
%template(WarningCommandDataReader) FastddsDataReader<ros2_interface::msg::WarningCommandPubSubType>;
