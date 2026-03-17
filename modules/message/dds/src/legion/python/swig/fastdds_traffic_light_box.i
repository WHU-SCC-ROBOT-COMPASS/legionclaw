%module fastdds_traffic_light_box 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TrafficLightBoxPubSubTypes.h"
%}
%include "std_string.i"
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
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class TrafficLightBox
    {
    public:
        TrafficLightBox();
        ~TrafficLightBox();

        void x(
            int _x);
        int x() const;
        void y(
            int _y);
        int y() const;
        void width(
            int _width);
        int width() const;
        void height(
            int _height);
        int height() const;
        void color(
            int _color);
        int color() const;
        void selected(
            bool _selected);
        bool selected() const;
        void camera_name(
           const std::string &_camera_name);
        std::string camera_name() const;
    };



    class TrafficLightBoxPubSubType
    {
    public:
        typedef TrafficLightBox type;

        TrafficLightBoxPubSubType();
        ~TrafficLightBoxPubSubType();
    };

}
}


%template(TrafficLightBoxDataWriter) FastddsDataWriter<ros2_interface::msg::TrafficLightBoxPubSubType>;
%template(TrafficLightBoxDataReader) FastddsDataReader<ros2_interface::msg::TrafficLightBoxPubSubType>;
