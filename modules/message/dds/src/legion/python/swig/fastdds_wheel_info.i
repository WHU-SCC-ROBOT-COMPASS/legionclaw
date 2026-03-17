%module fastdds_wheel_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "WheelInfoPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
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
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class WheelInfo
    {
    public:
        WheelInfo();
        ~WheelInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void moving_status(
            int _moving_status);
        int moving_status() const;
        void steer_driving_mode(
            int _steer_driving_mode);
        int steer_driving_mode() const;
        void steering_value(
            double _steering_value);
        double steering_value() const;
        void steering_torque_nm(
            double _steering_torque_nm);
        double steering_torque_nm() const;
        void steering_rate_dps(
            double _steering_rate_dps);
        double steering_rate_dps() const;
        void speed_mps(
            double _speed_mps);
        double speed_mps() const;
        void veh_spd_vld(
            bool _veh_spd_vld);
        bool veh_spd_vld() const;
        void gear(
            int _gear);
        int gear() const;
        void gear_vld(
            bool _gear_vld);
        bool gear_vld() const;
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
        void yaw_rate(
            double _yaw_rate);
        double yaw_rate() const;
        void wss_fl_edges_sum(
            int _wss_fl_edges_sum);
        int wss_fl_edges_sum() const;
        void wss_fr_edges_sum(
            int _wss_fr_edges_sum);
        int wss_fr_edges_sum() const;
        void wss_rl_edges_sum(
            int _wss_rl_edges_sum);
        int wss_rl_edges_sum() const;
        void wss_rr_edges_sum(
            int _wss_rr_edges_sum);
        int wss_rr_edges_sum() const;
        void wss_fl_edges_sum_vld(
            bool _wss_fl_edges_sum_vld);
        bool wss_fl_edges_sum_vld() const;
        void wss_fr_edges_sum_vld(
            bool _wss_fr_edges_sum_vld);
        bool wss_fr_edges_sum_vld() const;
        void wss_rl_edges_sum_vld(
            bool _wss_rl_edges_sum_vld);
        bool wss_rl_edges_sum_vld() const;
        void wss_rr_edges_sum_vld(
            bool _wss_rr_edges_sum_vld);
        bool wss_rr_edges_sum_vld() const;
        void veh_lat_accel(
            double _veh_lat_accel);
        double veh_lat_accel() const;
        void veh_lgt_accel(
            double _veh_lgt_accel);
        double veh_lgt_accel() const;
        void veh_lat_accel_vld(
            bool _veh_lat_accel_vld);
        bool veh_lat_accel_vld() const;
        void veh_lgt_accel_vld(
            bool _veh_lgt_accel_vld);
        bool veh_lgt_accel_vld() const;
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



    class WheelInfoPubSubType
    {
    public:
        typedef WheelInfo type;

        WheelInfoPubSubType();
        ~WheelInfoPubSubType();
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


%template(WheelInfoDataWriter) FastddsDataWriter<ros2_interface::msg::WheelInfoPubSubType>;
%template(WheelInfoDataReader) FastddsDataReader<ros2_interface::msg::WheelInfoPubSubType>;
