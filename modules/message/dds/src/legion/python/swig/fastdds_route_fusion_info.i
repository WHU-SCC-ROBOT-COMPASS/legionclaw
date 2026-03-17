%module fastdds_route_fusion_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RouteFusionInfoPubSubTypes.h"
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
    class RouteFusionInfo
    {
    public:
        RouteFusionInfo();
        ~RouteFusionInfo();

        void fusion_flag(
            int _fusion_flag);
        int fusion_flag() const;
        void fusion_reason(
           const std::string &_fusion_reason);
        std::string fusion_reason() const;
    };



    class RouteFusionInfoPubSubType
    {
    public:
        typedef RouteFusionInfo type;

        RouteFusionInfoPubSubType();
        ~RouteFusionInfoPubSubType();
    };

}
}


%template(RouteFusionInfoDataWriter) FastddsDataWriter<ros2_interface::msg::RouteFusionInfoPubSubType>;
%template(RouteFusionInfoDataReader) FastddsDataReader<ros2_interface::msg::RouteFusionInfoPubSubType>;
