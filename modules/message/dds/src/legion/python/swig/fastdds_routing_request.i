%module fastdds_routing_request 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "KeyPointPubSubTypes.h"
#include "RoutingRequestPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int16_t;
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
    %template(VectorKeyPoint) vector<ros2_interface::msg::KeyPoint>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class KeyPoint
    {
    public:
        KeyPoint();
        ~KeyPoint();

        void id(
            int _id);
        int id() const;
        void latitude(
            double _latitude);
        double latitude() const;
        void longitude(
            double _longitude);
        double longitude() const;
        void ele(
            double _ele);
        double ele() const;
        void heading(
            double _heading);
        double heading() const;
        void name(
           const std::string &_name);
        std::string name() const;
    };

    class RoutingRequest
    {
    public:
        RoutingRequest();
        ~RoutingRequest();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void request_source(
           const std::string &_request_source);
        std::string request_source() const;
        void request_type(
            int _request_type);
        int request_type() const;
        void num_of_kp(
            int _num_of_kp);
        int num_of_kp() const;
        void key_point_list(
            const std::vector<ros2_interface::msg::KeyPoint> &_key_point_list);
        const std::vector<ros2_interface::msg::KeyPoint>& key_point_list() const;
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



    class KeyPointPubSubType
    {
    public:
        typedef KeyPoint type;

        KeyPointPubSubType();
        ~KeyPointPubSubType();
    };

    class RoutingRequestPubSubType
    {
    public:
        typedef RoutingRequest type;

        RoutingRequestPubSubType();
        ~RoutingRequestPubSubType();
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


%template(RoutingRequestDataWriter) FastddsDataWriter<ros2_interface::msg::RoutingRequestPubSubType>;
%template(RoutingRequestDataReader) FastddsDataReader<ros2_interface::msg::RoutingRequestPubSubType>;
