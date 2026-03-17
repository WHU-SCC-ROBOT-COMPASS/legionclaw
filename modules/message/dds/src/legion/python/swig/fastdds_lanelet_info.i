%module fastdds_lanelet_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "LaneletInfoPubSubTypes.h"
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
    class LaneletInfo
    {
    public:
        LaneletInfo();
        ~LaneletInfo();

        void lanelet_id(
            int _lanelet_id);
        int lanelet_id() const;
        void length(
            double _length);
        double length() const;
    };



    class LaneletInfoPubSubType
    {
    public:
        typedef LaneletInfo type;

        LaneletInfoPubSubType();
        ~LaneletInfoPubSubType();
    };

}
}


%template(LaneletInfoDataWriter) FastddsDataWriter<ros2_interface::msg::LaneletInfoPubSubType>;
%template(LaneletInfoDataReader) FastddsDataReader<ros2_interface::msg::LaneletInfoPubSubType>;
