%module fastdds_radar_state_error 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RadarStateErrorPubSubTypes.h"
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
    class RadarStateError
    {
    public:
        RadarStateError();
        ~RadarStateError();

        void persistent_error(
            int _persistent_error);
        int persistent_error() const;
        void temporary_error(
            int _temporary_error);
        int temporary_error() const;
        void interference_error(
            int _interference_error);
        int interference_error() const;
        void temperature_error(
            int _temperature_error);
        int temperature_error() const;
        void voltage_error(
            int _voltage_error);
        int voltage_error() const;
        void block_error(
            int _block_error);
        int block_error() const;
        void broadcast_error(
            int _broadcast_error);
        int broadcast_error() const;
        void electric_axis_error(
            int _electric_axis_error);
        int electric_axis_error() const;
        void config_error(
            int _config_error);
        int config_error() const;
        void calibration_error(
            int _calibration_error);
        int calibration_error() const;
    };



    class RadarStateErrorPubSubType
    {
    public:
        typedef RadarStateError type;

        RadarStateErrorPubSubType();
        ~RadarStateErrorPubSubType();
    };

}
}


%template(RadarStateErrorDataWriter) FastddsDataWriter<ros2_interface::msg::RadarStateErrorPubSubType>;
%template(RadarStateErrorDataReader) FastddsDataReader<ros2_interface::msg::RadarStateErrorPubSubType>;
