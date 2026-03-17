%module fastdds_status 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "StatusPubSubTypes.h"
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
    class Status
    {
    public:
        Status();
        ~Status();

        void error_code(
            int _error_code);
        int error_code() const;
        void msg(
           const std::string &_msg);
        std::string msg() const;
    };



    class StatusPubSubType
    {
    public:
        typedef Status type;

        StatusPubSubType();
        ~StatusPubSubType();
    };

}
}


%template(StatusDataWriter) FastddsDataWriter<ros2_interface::msg::StatusPubSubType>;
%template(StatusDataReader) FastddsDataReader<ros2_interface::msg::StatusPubSubType>;
