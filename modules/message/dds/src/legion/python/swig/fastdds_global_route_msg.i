%module fastdds_global_route_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "GlobalRouteMsgPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "LaneletInfoPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef int int32_t;



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
    %template(VectorLaneletInfo) vector<ros2_interface::msg::LaneletInfo>;
    %template(VectorLaneletInfo) vector<ros2_interface::msg::LaneletInfo>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class GlobalRouteMsg
    {
    public:
        GlobalRouteMsg();
        ~GlobalRouteMsg();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void route(
            const std::vector<ros2_interface::msg::LaneletInfo> &_route);
        const std::vector<ros2_interface::msg::LaneletInfo>& route() const;
        void current_lanelet(
                const ros2_interface::msg::LaneletInfo &_current_lanelet);
        const ros2_interface::msg::LaneletInfo &current_lanelet() const;
        void total_mileage(
            double _total_mileage);
        double total_mileage() const;
        void cur_mileage(
            double _cur_mileage);
        double cur_mileage() const;
        void cur_slice(
            const std::vector<ros2_interface::msg::LaneletInfo> &_cur_slice);
        const std::vector<ros2_interface::msg::LaneletInfo>& cur_slice() const;
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

    class LaneletInfo
    {
    public:
        LaneletInfo();
        ~LaneletInfo();

        void lanelet_id(
            int _lanelet_id);
        int lanelet_id() const;
        void length(
            double _length);
        double length() const;
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



    class GlobalRouteMsgPubSubType
    {
    public:
        typedef GlobalRouteMsg type;

        GlobalRouteMsgPubSubType();
        ~GlobalRouteMsgPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class LaneletInfoPubSubType
    {
    public:
        typedef LaneletInfo type;

        LaneletInfoPubSubType();
        ~LaneletInfoPubSubType();
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


%template(GlobalRouteMsgDataWriter) FastddsDataWriter<ros2_interface::msg::GlobalRouteMsgPubSubType>;
%template(GlobalRouteMsgDataReader) FastddsDataReader<ros2_interface::msg::GlobalRouteMsgPubSubType>;
