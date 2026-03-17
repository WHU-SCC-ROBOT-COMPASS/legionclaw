%module fastdds_lane_line_cubic_curve 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "LaneLineCubicCurvePubSubTypes.h"
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
    class LaneLineCubicCurve
    {
    public:
        LaneLineCubicCurve();
        ~LaneLineCubicCurve();

        void start_x(
            double _start_x);
        double start_x() const;
        void end_x(
            double _end_x);
        double end_x() const;
        void a(
            double _a);
        double a() const;
        void b(
            double _b);
        double b() const;
        void c(
            double _c);
        double c() const;
        void d(
            double _d);
        double d() const;
    };



    class LaneLineCubicCurvePubSubType
    {
    public:
        typedef LaneLineCubicCurve type;

        LaneLineCubicCurvePubSubType();
        ~LaneLineCubicCurvePubSubType();
    };

}
}


%template(LaneLineCubicCurveDataWriter) FastddsDataWriter<ros2_interface::msg::LaneLineCubicCurvePubSubType>;
%template(LaneLineCubicCurveDataReader) FastddsDataReader<ros2_interface::msg::LaneLineCubicCurvePubSubType>;
