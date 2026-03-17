%module fastdds_planning_cmd 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PlanningCmdPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
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
    class PlanningCmd
    {
    public:
        PlanningCmd();
        ~PlanningCmd();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
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



    class PlanningCmdPubSubType
    {
    public:
        typedef PlanningCmd type;

        PlanningCmdPubSubType();
        ~PlanningCmdPubSubType();
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


%template(PlanningCmdDataWriter) FastddsDataWriter<ros2_interface::msg::PlanningCmdPubSubType>;
%template(PlanningCmdDataReader) FastddsDataReader<ros2_interface::msg::PlanningCmdPubSubType>;
