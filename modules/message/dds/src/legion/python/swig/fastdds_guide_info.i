%module fastdds_guide_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "GuideRoadPubSubTypes.h"
#include "GuideInfoPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "CurvatureInfoPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int64_t;
 typedef int int8_t;
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

    class GuideInfo
    {
    public:
        GuideInfo();
        ~GuideInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void next_dis(
            double _next_dis);
        double next_dis() const;
        void current_road(
                const ros2_interface::msg::GuideRoad &_current_road);
        const ros2_interface::msg::GuideRoad &current_road() const;
        void next_road(
                const ros2_interface::msg::GuideRoad &_next_road);
        const ros2_interface::msg::GuideRoad &next_road() const;
        void round_status(
            int _round_status);
        int round_status() const;
        void intersection_status(
            int _intersection_status);
        int intersection_status() const;
        void roads_status(
            int _roads_status);
        int roads_status() const;
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

    class GuideInfoPubSubType
    {
    public:
        typedef GuideInfo type;

        GuideInfoPubSubType();
        ~GuideInfoPubSubType();
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

    class CurvatureInfoPubSubType
    {
    public:
        typedef CurvatureInfo type;

        CurvatureInfoPubSubType();
        ~CurvatureInfoPubSubType();
    };

}
}


%template(GuideInfoDataWriter) FastddsDataWriter<ros2_interface::msg::GuideInfoPubSubType>;
%template(GuideInfoDataReader) FastddsDataReader<ros2_interface::msg::GuideInfoPubSubType>;
