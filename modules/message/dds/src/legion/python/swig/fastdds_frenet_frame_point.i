%module fastdds_frenet_frame_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "FrenetFramePointPubSubTypes.h"
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
    class FrenetFramePoint
    {
    public:
        FrenetFramePoint();
        ~FrenetFramePoint();

        void s(
            double _s);
        double s() const;
        void l(
            double _l);
        double l() const;
        void dl(
            double _dl);
        double dl() const;
        void ddl(
            double _ddl);
        double ddl() const;
    };



    class FrenetFramePointPubSubType
    {
    public:
        typedef FrenetFramePoint type;

        FrenetFramePointPubSubType();
        ~FrenetFramePointPubSubType();
    };

}
}


%template(FrenetFramePointDataWriter) FastddsDataWriter<ros2_interface::msg::FrenetFramePointPubSubType>;
%template(FrenetFramePointDataReader) FastddsDataReader<ros2_interface::msg::FrenetFramePointPubSubType>;
