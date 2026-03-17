%module fastdds_curvature_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "CurvatureInfoPubSubTypes.h"
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
    class CurvatureInfo
    {
    public:
        CurvatureInfo();
        ~CurvatureInfo();

        void offset(
            double _offset);
        double offset() const;
        void value(
            double _value);
        double value() const;
    };



    class CurvatureInfoPubSubType
    {
    public:
        typedef CurvatureInfo type;

        CurvatureInfoPubSubType();
        ~CurvatureInfoPubSubType();
    };

}
}


%template(CurvatureInfoDataWriter) FastddsDataWriter<ros2_interface::msg::CurvatureInfoPubSubType>;
%template(CurvatureInfoDataReader) FastddsDataReader<ros2_interface::msg::CurvatureInfoPubSubType>;
