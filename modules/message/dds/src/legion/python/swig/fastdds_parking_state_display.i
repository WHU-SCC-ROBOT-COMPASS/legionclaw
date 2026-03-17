%module fastdds_parking_state_display 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ParkingStateDisplayPubSubTypes.h"
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
    class ParkingStateDisplay
    {
    public:
        ParkingStateDisplay();
        ~ParkingStateDisplay();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void parking_type(
            int _parking_type);
        int parking_type() const;
        void moves_counter(
            int _moves_counter);
        int moves_counter() const;
        void remian_distance(
            double _remian_distance);
        double remian_distance() const;
        void display_info(
            int _display_info);
        int display_info() const;
        void distance_to_leader_obj(
            double _distance_to_leader_obj);
        double distance_to_leader_obj() const;
        void state(
            int _state);
        int state() const;
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



    class ParkingStateDisplayPubSubType
    {
    public:
        typedef ParkingStateDisplay type;

        ParkingStateDisplayPubSubType();
        ~ParkingStateDisplayPubSubType();
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


%template(ParkingStateDisplayDataWriter) FastddsDataWriter<ros2_interface::msg::ParkingStateDisplayPubSubType>;
%template(ParkingStateDisplayDataReader) FastddsDataReader<ros2_interface::msg::ParkingStateDisplayPubSubType>;
