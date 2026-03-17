%module fastdds_traffic_light 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImageRectPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "TrafficLightPubSubTypes.h"
%}
%include "std_vector.i"
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
    %template(Vectorint) vector<int>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class ImageRect
    {
    public:
        ImageRect();
        ~ImageRect();

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
    };

    class Point3D
    {
    public:
        Point3D();
        ~Point3D();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
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

    class TrafficLight
    {
    public:
        TrafficLight();
        ~TrafficLight();

        void color(
            int _color);
        int color() const;
        void id(
            int _id);
        int id() const;
        void type(
            int _type);
        int type() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void light_rect(
                const ros2_interface::msg::ImageRect &_light_rect);
        const ros2_interface::msg::ImageRect &light_rect() const;
        void position(
                const ros2_interface::msg::Point3D &_position);
        const ros2_interface::msg::Point3D &position() const;
        void distance(
            double _distance);
        double distance() const;
        void light_lanes(
            const std::vector<int> &_light_lanes);
        const std::vector<int>& light_lanes() const;
        void tracking_time(
            double _tracking_time);
        double tracking_time() const;
        void blink(
            bool _blink);
        bool blink() const;
        void blinking_time(
            double _blinking_time);
        double blinking_time() const;
        void remaining_time(
            double _remaining_time);
        double remaining_time() const;
        void create_time(
                const ros2_interface::msg::Time &_create_time);
        const ros2_interface::msg::Time &create_time() const;
    };



    class ImageRectPubSubType
    {
    public:
        typedef ImageRect type;

        ImageRectPubSubType();
        ~ImageRectPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class TrafficLightPubSubType
    {
    public:
        typedef TrafficLight type;

        TrafficLightPubSubType();
        ~TrafficLightPubSubType();
    };

}
}


%template(TrafficLightDataWriter) FastddsDataWriter<ros2_interface::msg::TrafficLightPubSubType>;
%template(TrafficLightDataReader) FastddsDataReader<ros2_interface::msg::TrafficLightPubSubType>;
