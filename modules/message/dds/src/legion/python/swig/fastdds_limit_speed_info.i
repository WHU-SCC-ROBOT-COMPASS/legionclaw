%module fastdds_limit_speed_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "LimitSpeedInfoPubSubTypes.h"
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
    class LimitSpeedInfo
    {
    public:
        LimitSpeedInfo();
        ~LimitSpeedInfo();

        void limitspeed_valid_flag(
            int _limitspeed_valid_flag);
        int limitspeed_valid_flag() const;
        void limit_speed(
            double _limit_speed);
        double limit_speed() const;
        void limit_distance(
            double _limit_distance);
        double limit_distance() const;
    };



    class LimitSpeedInfoPubSubType
    {
    public:
        typedef LimitSpeedInfo type;

        LimitSpeedInfoPubSubType();
        ~LimitSpeedInfoPubSubType();
    };

}
}


%template(LimitSpeedInfoDataWriter) FastddsDataWriter<ros2_interface::msg::LimitSpeedInfoPubSubType>;
%template(LimitSpeedInfoDataReader) FastddsDataReader<ros2_interface::msg::LimitSpeedInfoPubSubType>;
