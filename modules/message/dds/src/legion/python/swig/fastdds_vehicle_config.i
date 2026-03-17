%module fastdds_vehicle_config 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "VehicleParamPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "VehicleConfigPubSubTypes.h"
%}
%include "std_string.i"
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
    class VehicleParam
    {
    public:
        VehicleParam();
        ~VehicleParam();

        void brand(
           const std::string &_brand);
        std::string brand() const;
        void steer_mode(
            int _steer_mode);
        int steer_mode() const;
        void length(
            double _length);
        double length() const;
        void width(
            double _width);
        double width() const;
        void height(
            double _height);
        double height() const;
        void mass_fl(
            double _mass_fl);
        double mass_fl() const;
        void mass_fr(
            double _mass_fr);
        double mass_fr() const;
        void mass_rl(
            double _mass_rl);
        double mass_rl() const;
        void mass_rr(
            double _mass_rr);
        double mass_rr() const;
        void wheel_radius(
            double _wheel_radius);
        double wheel_radius() const;
        void wheelbase(
            double _wheelbase);
        double wheelbase() const;
        void front_edge_to_center(
            double _front_edge_to_center);
        double front_edge_to_center() const;
        void back_edge_to_center(
            double _back_edge_to_center);
        double back_edge_to_center() const;
        void lf(
            double _lf);
        double lf() const;
        void lr(
            double _lr);
        double lr() const;
        void cf(
            double _cf);
        double cf() const;
        void cr(
            double _cr);
        double cr() const;
        void steer_ratio(
            double _steer_ratio);
        double steer_ratio() const;
        void rolling_coefficient(
            double _rolling_coefficient);
        double rolling_coefficient() const;
        void air_density(
            double _air_density);
        double air_density() const;
        void air_damping_coefficient(
            double _air_damping_coefficient);
        double air_damping_coefficient() const;
        void max_front_steer_angle(
            double _max_front_steer_angle);
        double max_front_steer_angle() const;
        void min_front_steer_angle(
            double _min_front_steer_angle);
        double min_front_steer_angle() const;
        void max_rear_steer_angle(
            double _max_rear_steer_angle);
        double max_rear_steer_angle() const;
        void min_rear_steer_angle(
            double _min_rear_steer_angle);
        double min_rear_steer_angle() const;
        void speed_limit(
            double _speed_limit);
        double speed_limit() const;
        void max_brake_value(
            double _max_brake_value);
        double max_brake_value() const;
        void min_brake_value(
            double _min_brake_value);
        double min_brake_value() const;
        void max_accel_value(
            double _max_accel_value);
        double max_accel_value() const;
        void min_accel_value(
            double _min_accel_value);
        double min_accel_value() const;
        void speed_deadzone(
            double _speed_deadzone);
        double speed_deadzone() const;
        void standstill_acceleration(
            double _standstill_acceleration);
        double standstill_acceleration() const;
        void max_front_steer_angle_rate(
            double _max_front_steer_angle_rate);
        double max_front_steer_angle_rate() const;
        void max_rear_steer_angle_rate(
            double _max_rear_steer_angle_rate);
        double max_rear_steer_angle_rate() const;
        void max_abs_speed_when_stopped(
            double _max_abs_speed_when_stopped);
        double max_abs_speed_when_stopped() const;
        void max_abs_speed_when_stopped_duration(
            double _max_abs_speed_when_stopped_duration);
        double max_abs_speed_when_stopped_duration() const;
        void brake_value_when_gear_transitioning(
            double _brake_value_when_gear_transitioning);
        double brake_value_when_gear_transitioning() const;
        void accel_deadzone(
            double _accel_deadzone);
        double accel_deadzone() const;
        void brake_deadzone(
            double _brake_deadzone);
        double brake_deadzone() const;
        void acceleration_in_idle(
            double _acceleration_in_idle);
        double acceleration_in_idle() const;
        void deceleration_in_idle(
            double _deceleration_in_idle);
        double deceleration_in_idle() const;
        void max_acceleration_jerk(
            double _max_acceleration_jerk);
        double max_acceleration_jerk() const;
        void max_acceleration(
            double _max_acceleration);
        double max_acceleration() const;
        void max_deceleration(
            double _max_deceleration);
        double max_deceleration() const;
        void min_turning_radius(
            double _min_turning_radius);
        double min_turning_radius() const;
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

    class VehicleConfig
    {
    public:
        VehicleConfig();
        ~VehicleConfig();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void vehicle_param(
                const ros2_interface::msg::VehicleParam &_vehicle_param);
        const ros2_interface::msg::VehicleParam &vehicle_param() const;
    };



    class VehicleParamPubSubType
    {
    public:
        typedef VehicleParam type;

        VehicleParamPubSubType();
        ~VehicleParamPubSubType();
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

    class VehicleConfigPubSubType
    {
    public:
        typedef VehicleConfig type;

        VehicleConfigPubSubType();
        ~VehicleConfigPubSubType();
    };

}
}


%template(VehicleConfigDataWriter) FastddsDataWriter<ros2_interface::msg::VehicleConfigPubSubType>;
%template(VehicleConfigDataReader) FastddsDataReader<ros2_interface::msg::VehicleConfigPubSubType>;
