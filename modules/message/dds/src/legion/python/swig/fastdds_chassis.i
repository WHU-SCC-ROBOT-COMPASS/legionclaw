%module fastdds_chassis 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ChassisPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
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
    %template(VectorErrorCode) vector<int>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Chassis
    {
    public:
        Chassis();
        ~Chassis();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void moving_status(
            int _moving_status);
        int moving_status() const;
        void driving_mode(
            int _driving_mode);
        int driving_mode() const;
        void steer_driving_mode(
            int _steer_driving_mode);
        int steer_driving_mode() const;
        void steering_status(
            int _steering_status);
        int steering_status() const;
        void front_steering_value(
            double _front_steering_value);
        double front_steering_value() const;
        void rear_steering_value(
            double _rear_steering_value);
        double rear_steering_value() const;
        void steering_torque_nm(
            double _steering_torque_nm);
        double steering_torque_nm() const;
        void front_steering_rate_dps(
            double _front_steering_rate_dps);
        double front_steering_rate_dps() const;
        void rear_steering_rate_dps(
            double _rear_steering_rate_dps);
        double rear_steering_rate_dps() const;
        void accel_driving_mode(
            int _accel_driving_mode);
        int accel_driving_mode() const;
        void accel_status(
            int _accel_status);
        int accel_status() const;
        void accel_value(
            double _accel_value);
        double accel_value() const;
        void brake_driving_mode(
            int _brake_driving_mode);
        int brake_driving_mode() const;
        void brake_status(
            int _brake_status);
        int brake_status() const;
        void brake_value(
            double _brake_value);
        double brake_value() const;
        void backup_brake_driving_mode(
            int _backup_brake_driving_mode);
        int backup_brake_driving_mode() const;
        void backup_brake_status(
            int _backup_brake_status);
        int backup_brake_status() const;
        void backup_brake_value(
            double _backup_brake_value);
        double backup_brake_value() const;
        void epb_driving_mode(
            int _epb_driving_mode);
        int epb_driving_mode() const;
        void epb_status(
            int _epb_status);
        int epb_status() const;
        void epb_level(
            int _epb_level);
        int epb_level() const;
        void engine_status(
            int _engine_status);
        int engine_status() const;
        void engine_rpm(
            double _engine_rpm);
        double engine_rpm() const;
        void engine_torque(
            double _engine_torque);
        double engine_torque() const;
        void speed_mps(
            double _speed_mps);
        double speed_mps() const;
        void odometer_m(
            double _odometer_m);
        double odometer_m() const;
        void fuel_range_m(
            int _fuel_range_m);
        int fuel_range_m() const;
        void gear_driving_mode(
            int _gear_driving_mode);
        int gear_driving_mode() const;
        void gear_status(
            int _gear_status);
        int gear_status() const;
        void gear_location(
            int _gear_location);
        int gear_location() const;
        void driver_seat_belt(
            int _driver_seat_belt);
        int driver_seat_belt() const;
        void high_beam_status(
            int _high_beam_status);
        int high_beam_status() const;
        void low_beam_status(
            int _low_beam_status);
        int low_beam_status() const;
        void horn_status(
            int _horn_status);
        int horn_status() const;
        void turn_lamp_status(
            int _turn_lamp_status);
        int turn_lamp_status() const;
        void front_wiper_status(
            int _front_wiper_status);
        int front_wiper_status() const;
        void rear_wiper_status(
            int _rear_wiper_status);
        int rear_wiper_status() const;
        void position_lamp_status(
            int _position_lamp_status);
        int position_lamp_status() const;
        void front_fog_lamp_status(
            int _front_fog_lamp_status);
        int front_fog_lamp_status() const;
        void rear_fog_lamp_status(
            int _rear_fog_lamp_status);
        int rear_fog_lamp_status() const;
        void brake_lamp_status(
            int _brake_lamp_status);
        int brake_lamp_status() const;
        void alarm_lamp_status(
            int _alarm_lamp_status);
        int alarm_lamp_status() const;
        void lf_door_status(
            int _lf_door_status);
        int lf_door_status() const;
        void rf_door_status(
            int _rf_door_status);
        int rf_door_status() const;
        void lr_door_status(
            int _lr_door_status);
        int lr_door_status() const;
        void rr_door_status(
            int _rr_door_status);
        int rr_door_status() const;
        void rearview_mirror_status(
            int _rearview_mirror_status);
        int rearview_mirror_status() const;
        void trunk_status(
            int _trunk_status);
        int trunk_status() const;
        void engine_bay_door_status(
            int _engine_bay_door_status);
        int engine_bay_door_status() const;
        void wheel_direction_rr(
            int _wheel_direction_rr);
        int wheel_direction_rr() const;
        void wheel_spd_rr(
            double _wheel_spd_rr);
        double wheel_spd_rr() const;
        void wheel_direction_rl(
            int _wheel_direction_rl);
        int wheel_direction_rl() const;
        void wheel_spd_rl(
            double _wheel_spd_rl);
        double wheel_spd_rl() const;
        void wheel_direction_fr(
            int _wheel_direction_fr);
        int wheel_direction_fr() const;
        void wheel_spd_fr(
            double _wheel_spd_fr);
        double wheel_spd_fr() const;
        void wheel_direction_fl(
            int _wheel_direction_fl);
        int wheel_direction_fl() const;
        void wheel_spd_fl(
            double _wheel_spd_fl);
        double wheel_spd_fl() const;
        void is_tire_pressure_ok(
            int _is_tire_pressure_ok);
        int is_tire_pressure_ok() const;
        void is_tire_pressure_lf_valid(
            int _is_tire_pressure_lf_valid);
        int is_tire_pressure_lf_valid() const;
        void tire_pressure_lf(
            double _tire_pressure_lf);
        double tire_pressure_lf() const;
        void is_tire_pressure_rf_valid(
            int _is_tire_pressure_rf_valid);
        int is_tire_pressure_rf_valid() const;
        void tire_pressure_rf(
            double _tire_pressure_rf);
        double tire_pressure_rf() const;
        void is_tire_pressure_lr_valid(
            int _is_tire_pressure_lr_valid);
        int is_tire_pressure_lr_valid() const;
        void tire_pressure_lr(
            double _tire_pressure_lr);
        double tire_pressure_lr() const;
        void is_tire_pressure_rr_valid(
            int _is_tire_pressure_rr_valid);
        int is_tire_pressure_rr_valid() const;
        void tire_pressure_rr(
            double _tire_pressure_rr);
        double tire_pressure_rr() const;
        void battery_power_percentage(
            double _battery_power_percentage);
        double battery_power_percentage() const;
        void air_bag_status(
            int _air_bag_status);
        int air_bag_status() const;
        void charging_gun_status(
            int _charging_gun_status);
        int charging_gun_status() const;
        void vehicle_power_status(
            int _vehicle_power_status);
        int vehicle_power_status() const;
        void chassis_error_code(
            const std::vector<int> &_chassis_error_code);
        const std::vector<int>& chassis_error_code() const;
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



    class ChassisPubSubType
    {
    public:
        typedef Chassis type;

        ChassisPubSubType();
        ~ChassisPubSubType();
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


%template(ChassisDataWriter) FastddsDataWriter<ros2_interface::msg::ChassisPubSubType>;
%template(ChassisDataReader) FastddsDataReader<ros2_interface::msg::ChassisPubSubType>;
