%module fastdds_point_llh 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointLLHPubSubTypes.h"
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



    class PointLLHPubSubType
    {
    public:
        typedef PointLLH type;

        PointLLHPubSubType();
        ~PointLLHPubSubType();
    };

}
}


%template(PointLLHDataWriter) FastddsDataWriter<ros2_interface::msg::PointLLHPubSubType>;
%template(PointLLHDataReader) FastddsDataReader<ros2_interface::msg::PointLLHPubSubType>;
