%module fastdds_security_decision 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SecurityDecisionPubSubTypes.h"
#include "WarningCommandPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TrajectoryLimitCommandPubSubTypes.h"
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
    class SecurityDecision
    {
    public:
        SecurityDecision();
        ~SecurityDecision();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void warning_command(
                const ros2_interface::msg::WarningCommand &_warning_command);
        const ros2_interface::msg::WarningCommand &warning_command() const;
        void brake_report(
            int _brake_report);
        int brake_report() const;
        void change_lane_command(
            int _change_lane_command);
        int change_lane_command() const;
        void trajectory_limit_command(
                const ros2_interface::msg::TrajectoryLimitCommand &_trajectory_limit_command);
        const ros2_interface::msg::TrajectoryLimitCommand &trajectory_limit_command() const;
        void park_command(
            int _park_command);
        int park_command() const;
        void drivingmode_report(
            int _drivingmode_report);
        int drivingmode_report() const;
    };

    class WarningCommand
    {
    public:
        WarningCommand();
        ~WarningCommand();

        void sound_alarm(
            int _sound_alarm);
        int sound_alarm() const;
        void light_alarm(
            int _light_alarm);
        int light_alarm() const;
        void media_alarm(
            int _media_alarm);
        int media_alarm() const;
        void motion_alarm(
            int _motion_alarm);
        int motion_alarm() const;
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

    class TrajectoryLimitCommand
    {
    public:
        TrajectoryLimitCommand();
        ~TrajectoryLimitCommand();

        void speed_limit_enable(
            bool _speed_limit_enable);
        bool speed_limit_enable() const;
        void speed_limit(
            double _speed_limit);
        double speed_limit() const;
        void kappa_limit_enable(
            bool _kappa_limit_enable);
        bool kappa_limit_enable() const;
        void kappa_limit(
            double _kappa_limit);
        double kappa_limit() const;
    };



    class SecurityDecisionPubSubType
    {
    public:
        typedef SecurityDecision type;

        SecurityDecisionPubSubType();
        ~SecurityDecisionPubSubType();
    };

    class WarningCommandPubSubType
    {
    public:
        typedef WarningCommand type;

        WarningCommandPubSubType();
        ~WarningCommandPubSubType();
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

    class TrajectoryLimitCommandPubSubType
    {
    public:
        typedef TrajectoryLimitCommand type;

        TrajectoryLimitCommandPubSubType();
        ~TrajectoryLimitCommandPubSubType();
    };

}
}


%template(SecurityDecisionDataWriter) FastddsDataWriter<ros2_interface::msg::SecurityDecisionPubSubType>;
%template(SecurityDecisionDataReader) FastddsDataReader<ros2_interface::msg::SecurityDecisionPubSubType>;
