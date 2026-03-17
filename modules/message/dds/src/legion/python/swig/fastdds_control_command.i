%module fastdds_control_command 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ControlCommandPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int8_t;
 typedef unsigned int uint32_t;



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
    class ControlCommand
    {
    public:
        ControlCommand();
        ~ControlCommand();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void steer_driving_mode(
            int _steer_driving_mode);
        int steer_driving_mode() const;
        void front_steering_target(
            double _front_steering_target);
        double front_steering_target() const;
        void rear_steering_target(
            double _rear_steering_target);
        double rear_steering_target() const;
        void front_steering_rate(
            double _front_steering_rate);
        double front_steering_rate() const;
        void rear_steering_rate(
            double _rear_steering_rate);
        double rear_steering_rate() const;
        void accel_driving_mode(
            int _accel_driving_mode);
        int accel_driving_mode() const;
        void accel_value(
            double _accel_value);
        double accel_value() const;
        void brake_driving_mode(
            int _brake_driving_mode);
        int brake_driving_mode() const;
        void brake_value(
            double _brake_value);
        double brake_value() const;
        void backup_brake_driving_mode(
            int _backup_brake_driving_mode);
        int backup_brake_driving_mode() const;
        void backup_brake_value(
            double _backup_brake_value);
        double backup_brake_value() const;
        void epb_driving_mode(
            int _epb_driving_mode);
        int epb_driving_mode() const;
        void epb_level(
            int _epb_level);
        int epb_level() const;
        void gear_driving_mode(
            int _gear_driving_mode);
        int gear_driving_mode() const;
        void emergency_brake_enable(
            int _emergency_brake_enable);
        int emergency_brake_enable() const;
        void gear_location(
            int _gear_location);
        int gear_location() const;
        void speed(
            double _speed);
        double speed() const;
        void acceleration(
            double _acceleration);
        double acceleration() const;
        void turn_lamp_ctrl(
            int _turn_lamp_ctrl);
        int turn_lamp_ctrl() const;
        void high_beam_ctrl(
            int _high_beam_ctrl);
        int high_beam_ctrl() const;
        void low_beam_ctrl(
            int _low_beam_ctrl);
        int low_beam_ctrl() const;
        void horn_ctrl(
            int _horn_ctrl);
        int horn_ctrl() const;
        void front_wiper_ctrl(
            int _front_wiper_ctrl);
        int front_wiper_ctrl() const;
        void rear_wiper_ctrl(
            int _rear_wiper_ctrl);
        int rear_wiper_ctrl() const;
        void position_lamp_ctrl(
            int _position_lamp_ctrl);
        int position_lamp_ctrl() const;
        void front_fog_lamp_ctrl(
            int _front_fog_lamp_ctrl);
        int front_fog_lamp_ctrl() const;
        void rear_fog_lamp_ctrl(
            int _rear_fog_lamp_ctrl);
        int rear_fog_lamp_ctrl() const;
        void brake_lamp_ctrl(
            int _brake_lamp_ctrl);
        int brake_lamp_ctrl() const;
        void alarm_lamp_ctrl(
            int _alarm_lamp_ctrl);
        int alarm_lamp_ctrl() const;
        void lf_door_ctrl(
            int _lf_door_ctrl);
        int lf_door_ctrl() const;
        void rf_door_ctrl(
            int _rf_door_ctrl);
        int rf_door_ctrl() const;
        void lr_door_ctrl(
            int _lr_door_ctrl);
        int lr_door_ctrl() const;
        void rr_door_ctrl(
            int _rr_door_ctrl);
        int rr_door_ctrl() const;
    };

    class Time
    {
    public:
        Time();
        ~Time();

        void sec(
            int _sec);
        int sec() const;
        void nsec(
            int _nsec);
        int nsec() const;
    };

    class Header
    {
    public:
        Header();
        ~Header();

        void seq(
            int _seq);
        int seq() const;
        void stamp(
                const ros2_interface::msg::Time &_stamp);
        const ros2_interface::msg::Time &stamp() const;
        void frame_id(
           const std::string &_frame_id);
        std::string frame_id() const;
    };



    class ControlCommandPubSubType
    {
    public:
        typedef ControlCommand type;

        ControlCommandPubSubType();
        ~ControlCommandPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

}
}


%template(ControlCommandDataWriter) FastddsDataWriter<ros2_interface::msg::ControlCommandPubSubType>;
%template(ControlCommandDataReader) FastddsDataReader<ros2_interface::msg::ControlCommandPubSubType>;
