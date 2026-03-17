%module fastdds_radar_state 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RadarStateErrorPubSubTypes.h"
#include "RadarStatePubSubTypes.h"
#include "RadarStateModePubSubTypes.h"
%}
 typedef unsigned int uint8_t;
 typedef unsigned int uint16_t;



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

    class RadarState
    {
    public:
        RadarState();
        ~RadarState();

        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void nvm_read_status(
            int _nvm_read_status);
        int nvm_read_status() const;
        void nvm_write_status(
            int _nvm_write_status);
        int nvm_write_status() const;
        void radar_state_error(
                const ros2_interface::msg::RadarStateError &_radar_state_error);
        const ros2_interface::msg::RadarStateError &radar_state_error() const;
        void radar_state_mode(
                const ros2_interface::msg::RadarStateMode &_radar_state_mode);
        const ros2_interface::msg::RadarStateMode &radar_state_mode() const;
        void max_distance(
            int _max_distance);
        int max_distance() const;
        void sort_index(
            int _sort_index);
        int sort_index() const;
        void radar_power(
            int _radar_power);
        int radar_power() const;
        void ctl_relay(
            int _ctl_relay);
        int ctl_relay() const;
        void output_type(
            int _output_type);
        int output_type() const;
        void send_quality(
            int _send_quality);
        int send_quality() const;
        void send_extinfo(
            int _send_extinfo);
        int send_extinfo() const;
        void motion_rx_state(
            int _motion_rx_state);
        int motion_rx_state() const;
        void rcs_threshold(
            int _rcs_threshold);
        int rcs_threshold() const;
        void connector_direction(
            int _connector_direction);
        int connector_direction() const;
        void radar_position(
            int _radar_position);
        int radar_position() const;
        void hw_error(
            int _hw_error);
        int hw_error() const;
    };

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



    class RadarStateErrorPubSubType
    {
    public:
        typedef RadarStateError type;

        RadarStateErrorPubSubType();
        ~RadarStateErrorPubSubType();
    };

    class RadarStatePubSubType
    {
    public:
        typedef RadarState type;

        RadarStatePubSubType();
        ~RadarStatePubSubType();
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


%template(RadarStateDataWriter) FastddsDataWriter<ros2_interface::msg::RadarStatePubSubType>;
%template(RadarStateDataReader) FastddsDataReader<ros2_interface::msg::RadarStatePubSubType>;
