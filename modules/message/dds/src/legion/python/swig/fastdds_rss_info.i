%module fastdds_rss_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "RSSInfoPubSubTypes.h"
%}



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
    class RSSInfo
    {
    public:
        RSSInfo();
        ~RSSInfo();

        void is_rss_safe(
            bool _is_rss_safe);
        bool is_rss_safe() const;
        void cur_dist_lon(
            double _cur_dist_lon);
        double cur_dist_lon() const;
        void rss_safe_dist_lon(
            double _rss_safe_dist_lon);
        double rss_safe_dist_lon() const;
        void acc_lon_range_minimum(
            double _acc_lon_range_minimum);
        double acc_lon_range_minimum() const;
        void acc_lon_range_maximum(
            double _acc_lon_range_maximum);
        double acc_lon_range_maximum() const;
        void acc_lat_left_range_minimum(
            double _acc_lat_left_range_minimum);
        double acc_lat_left_range_minimum() const;
        void acc_lat_left_range_maximum(
            double _acc_lat_left_range_maximum);
        double acc_lat_left_range_maximum() const;
        void acc_lat_right_range_minimum(
            double _acc_lat_right_range_minimum);
        double acc_lat_right_range_minimum() const;
        void acc_lat_right_range_maximum(
            double _acc_lat_right_range_maximum);
        double acc_lat_right_range_maximum() const;
    };



    class RSSInfoPubSubType
    {
    public:
        typedef RSSInfo type;

        RSSInfoPubSubType();
        ~RSSInfoPubSubType();
    };

}
}


%template(RSSInfoDataWriter) FastddsDataWriter<ros2_interface::msg::RSSInfoPubSubType>;
%template(RSSInfoDataReader) FastddsDataReader<ros2_interface::msg::RSSInfoPubSubType>;
