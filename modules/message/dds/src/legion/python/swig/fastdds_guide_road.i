%module fastdds_guide_road 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "GuideRoadPubSubTypes.h"
#include "CurvatureInfoPubSubTypes.h"
%}
%include "std_vector.i"
 typedef int int64_t;
 typedef int int8_t;
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
    %template(VectorCurvatureInfo) vector<ros2_interface::msg::CurvatureInfo>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class GuideRoad
    {
    public:
        GuideRoad();
        ~GuideRoad();

        void road_id(
            int _road_id);
        int road_id() const;
        void road_type(
            int _road_type);
        int road_type() const;
        void turn_type(
            int _turn_type);
        int turn_type() const;
        void avg_curvature(
            double _avg_curvature);
        double avg_curvature() const;
        void curvature_size(
            int _curvature_size);
        int curvature_size() const;
        void curvature(
            const std::vector<ros2_interface::msg::CurvatureInfo> &_curvature);
        const std::vector<ros2_interface::msg::CurvatureInfo>& curvature() const;
    };

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



    class GuideRoadPubSubType
    {
    public:
        typedef GuideRoad type;

        GuideRoadPubSubType();
        ~GuideRoadPubSubType();
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


%template(GuideRoadDataWriter) FastddsDataWriter<ros2_interface::msg::GuideRoadPubSubType>;
%template(GuideRoadDataReader) FastddsDataReader<ros2_interface::msg::GuideRoadPubSubType>;
