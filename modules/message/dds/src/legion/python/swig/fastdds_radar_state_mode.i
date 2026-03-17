%module fastdds_radar_state_mode 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RadarStateModePubSubTypes.h"
%}
 typedef unsigned int uint8_t;



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
    class RadarStateMode
    {
    public:
        RadarStateMode();
        ~RadarStateMode();

        void can0_work_mode(
            int _can0_work_mode);
        int can0_work_mode() const;
        void can1_work_mode(
            int _can1_work_mode);
        int can1_work_mode() const;
        void dual_can_mode(
            int _dual_can_mode);
        int dual_can_mode() const;
        void timming_mode(
            int _timming_mode);
        int timming_mode() const;
        void power_mode(
            int _power_mode);
        int power_mode() const;
        void performance_mode(
            int _performance_mode);
        int performance_mode() const;
    };



    class RadarStateModePubSubType
    {
    public:
        typedef RadarStateMode type;

        RadarStateModePubSubType();
        ~RadarStateModePubSubType();
    };

}
}


%template(RadarStateModeDataWriter) FastddsDataWriter<ros2_interface::msg::RadarStateModePubSubType>;
%template(RadarStateModeDataReader) FastddsDataReader<ros2_interface::msg::RadarStateModePubSubType>;
