%module fastdds_hmi_vehicle_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointLLHPubSubTypes.h"
#include "PointENUPubSubTypes.h"
#include "HMIVehicleMsgPubSubTypes.h"
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
    class PointLLH
    {
    public:
        PointLLH();
        ~PointLLH();

        void lon(
            double _lon);
        double lon() const;
        void lat(
            double _lat);
        double lat() const;
        void height(
            double _height);
        double height() const;
    };

    class PointENU
    {
    public:
        PointENU();
        ~PointENU();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
    };

    class HMIVehicleMsg
    {
    public:
        HMIVehicleMsg();
        ~HMIVehicleMsg();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void driving_mode(
            int _driving_mode);
        int driving_mode() const;
        void gear_location(
            int _gear_location);
        int gear_location() const;
        void steering_value(
            double _steering_value);
        double steering_value() const;
        void accel_value(
            double _accel_value);
        double accel_value() const;
        void brake_value(
            double _brake_value);
        double brake_value() const;
        void speed_mps(
            double _speed_mps);
        double speed_mps() const;
        void position(
                const ros2_interface::msg::PointLLH &_position);
        const ros2_interface::msg::PointLLH &position() const;
        void utm_position(
                const ros2_interface::msg::PointENU &_utm_position);
        const ros2_interface::msg::PointENU &utm_position() const;
        void pitch(
            double _pitch);
        double pitch() const;
        void roll(
            double _roll);
        double roll() const;
        void heading(
            double _heading);
        double heading() const;
        void rtk_flag(
            int _rtk_flag);
        int rtk_flag() const;
        void origin_lat(
            double _origin_lat);
        double origin_lat() const;
        void origin_lon(
            double _origin_lon);
        double origin_lon() const;
        void current_ins_yaw(
            double _current_ins_yaw);
        double current_ins_yaw() const;
        void auto_safe(
            int _auto_safe);
        int auto_safe() const;
        void battery_power_percentage(
            double _battery_power_percentage);
        double battery_power_percentage() const;
        void charging_gun_status(
            int _charging_gun_status);
        int charging_gun_status() const;
        void chassis_error_code(
            const std::vector<int> &_chassis_error_code);
        const std::vector<int>& chassis_error_code() const;
        void brake_lamp_status(
            int _brake_lamp_status);
        int brake_lamp_status() const;
        void turn_lamp_status(
            int _turn_lamp_status);
        int turn_lamp_status() const;
        void mercator_position(
                const ros2_interface::msg::PointENU &_mercator_position);
        const ros2_interface::msg::PointENU &mercator_position() const;
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



    class PointLLHPubSubType
    {
    public:
        typedef PointLLH type;

        PointLLHPubSubType();
        ~PointLLHPubSubType();
    };

    class PointENUPubSubType
    {
    public:
        typedef PointENU type;

        PointENUPubSubType();
        ~PointENUPubSubType();
    };

    class HMIVehicleMsgPubSubType
    {
    public:
        typedef HMIVehicleMsg type;

        HMIVehicleMsgPubSubType();
        ~HMIVehicleMsgPubSubType();
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


%template(HMIVehicleMsgDataWriter) FastddsDataWriter<ros2_interface::msg::HMIVehicleMsgPubSubType>;
%template(HMIVehicleMsgDataReader) FastddsDataReader<ros2_interface::msg::HMIVehicleMsgPubSubType>;
