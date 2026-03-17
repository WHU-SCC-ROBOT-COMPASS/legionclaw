%module fastdds_holistic_path_prediction 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HolisticPathPredictionPubSubTypes.h"
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
    class HolisticPathPrediction
    {
    public:
        HolisticPathPrediction();
        ~HolisticPathPrediction();

        void hpp(
                const ros2_interface::msg::LaneLineCubicCurve &_hpp);
        const ros2_interface::msg::LaneLineCubicCurve &hpp() const;
        void planning_source(
            int _planning_source);
        int planning_source() const;
        void ego_lane_width(
            double _ego_lane_width);
        double ego_lane_width() const;
        void confidence(
            double _confidence);
        double confidence() const;
    };

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



    class HolisticPathPredictionPubSubType
    {
    public:
        typedef HolisticPathPrediction type;

        HolisticPathPredictionPubSubType();
        ~HolisticPathPredictionPubSubType();
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


%template(HolisticPathPredictionDataWriter) FastddsDataWriter<ros2_interface::msg::HolisticPathPredictionPubSubType>;
%template(HolisticPathPredictionDataReader) FastddsDataReader<ros2_interface::msg::HolisticPathPredictionPubSubType>;
