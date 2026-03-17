%module fastdds_traffic_light_debug 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TrafficLightBoxPubSubTypes.h"
#include "TrafficLightDebugPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
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
    %template(VectorTrafficLightBox) vector<ros2_interface::msg::TrafficLightBox>;
    %template(VectorTrafficLightBox) vector<ros2_interface::msg::TrafficLightBox>;
    %template(VectorTrafficLightBox) vector<ros2_interface::msg::TrafficLightBox>;
    %template(VectorTrafficLightBox) vector<ros2_interface::msg::TrafficLightBox>;
    %template(VectorTrafficLightBox) vector<ros2_interface::msg::TrafficLightBox>;
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



    class TrafficLightBoxPubSubType
    {
    public:
        typedef TrafficLightBox type;

        TrafficLightBoxPubSubType();
        ~TrafficLightBoxPubSubType();
    };

    class TrafficLightDebugPubSubType
    {
    public:
        typedef TrafficLightDebug type;

        TrafficLightDebugPubSubType();
        ~TrafficLightDebugPubSubType();
    };

}
}


%template(TrafficLightDebugDataWriter) FastddsDataWriter<ros2_interface::msg::TrafficLightDebugPubSubType>;
%template(TrafficLightDebugDataReader) FastddsDataReader<ros2_interface::msg::TrafficLightDebugPubSubType>;
