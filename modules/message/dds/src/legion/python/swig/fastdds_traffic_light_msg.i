%module fastdds_traffic_light_msg 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImageRectPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "TrafficLightMsgPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "TrafficLightDebugPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TrafficLightBoxPubSubTypes.h"
#include "TrafficLightPubSubTypes.h"
%}
%include "std_string.i"
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
    %template(VectorTrafficLight) vector<ros2_interface::msg::TrafficLight>;
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

    class TrafficLightMsg
    {
    public:
        TrafficLightMsg();
        ~TrafficLightMsg();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void traffic_light(
            const std::vector<ros2_interface::msg::TrafficLight> &_traffic_light);
        const std::vector<ros2_interface::msg::TrafficLight>& traffic_light() const;
        void traffic_light_debug(
                const ros2_interface::msg::TrafficLightDebug &_traffic_light_debug);
        const ros2_interface::msg::TrafficLightDebug &traffic_light_debug() const;
        void contain_lights(
            bool _contain_lights);
        bool contain_lights() const;
        void camera_id(
            int _camera_id);
        int camera_id() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
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

    class TrafficLightDebug
    {
    public:
        TrafficLightDebug();
        ~TrafficLightDebug();

        void cropbox(
                const ros2_interface::msg::TrafficLightBox &_cropbox);
        const ros2_interface::msg::TrafficLightBox &cropbox() const;
        void box(
            const std::vector<ros2_interface::msg::TrafficLightBox> &_box);
        const std::vector<ros2_interface::msg::TrafficLightBox>& box() const;
        void signal_num(
            int _signal_num);
        int signal_num() const;
        void valid_pos(
            int _valid_pos);
        int valid_pos() const;
        void ts_diff_pos(
            double _ts_diff_pos);
        double ts_diff_pos() const;
        void ts_diff_sys(
            double _ts_diff_sys);
        double ts_diff_sys() const;
        void project_error(
            int _project_error);
        int project_error() const;
        void distance_to_stop_line(
            double _distance_to_stop_line);
        double distance_to_stop_line() const;
        void camera_id(
            int _camera_id);
        int camera_id() const;
        void crop_roi(
            const std::vector<ros2_interface::msg::TrafficLightBox> &_crop_roi);
        const std::vector<ros2_interface::msg::TrafficLightBox>& crop_roi() const;
        void projected_roi(
            const std::vector<ros2_interface::msg::TrafficLightBox> &_projected_roi);
        const std::vector<ros2_interface::msg::TrafficLightBox>& projected_roi() const;
        void rectified_roi(
            const std::vector<ros2_interface::msg::TrafficLightBox> &_rectified_roi);
        const std::vector<ros2_interface::msg::TrafficLightBox>& rectified_roi() const;
        void debug_roi(
            const std::vector<ros2_interface::msg::TrafficLightBox> &_debug_roi);
        const std::vector<ros2_interface::msg::TrafficLightBox>& debug_roi() const;
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

    class TrafficLightMsgPubSubType
    {
    public:
        typedef TrafficLightMsg type;

        TrafficLightMsgPubSubType();
        ~TrafficLightMsgPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class TrafficLightDebugPubSubType
    {
    public:
        typedef TrafficLightDebug type;

        TrafficLightDebugPubSubType();
        ~TrafficLightDebugPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class TrafficLightBoxPubSubType
    {
    public:
        typedef TrafficLightBox type;

        TrafficLightBoxPubSubType();
        ~TrafficLightBoxPubSubType();
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


%template(TrafficLightMsgDataWriter) FastddsDataWriter<ros2_interface::msg::TrafficLightMsgPubSubType>;
%template(TrafficLightMsgDataReader) FastddsDataReader<ros2_interface::msg::TrafficLightMsgPubSubType>;
