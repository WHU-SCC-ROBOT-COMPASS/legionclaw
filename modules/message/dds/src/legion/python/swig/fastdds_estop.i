%module fastdds_estop 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "EStopPubSubTypes.h"
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
    class EStop
    {
    public:
        EStop();
        ~EStop();

        void is_estop(
            bool _is_estop);
        bool is_estop() const;
        void reason(
           const std::string &_reason);
        std::string reason() const;
    };



    class EStopPubSubType
    {
    public:
        typedef EStop type;

        EStopPubSubType();
        ~EStopPubSubType();
    };

}
}


%template(EStopDataWriter) FastddsDataWriter<ros2_interface::msg::EStopPubSubType>;
%template(EStopDataReader) FastddsDataReader<ros2_interface::msg::EStopPubSubType>;
