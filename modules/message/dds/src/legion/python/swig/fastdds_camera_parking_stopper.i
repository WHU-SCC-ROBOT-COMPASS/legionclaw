%module fastdds_camera_parking_stopper 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "BBox2DPubSubTypes.h"
#include "CameraParkingStopperPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int16_t;
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
    class BBox2D
    {
    public:
        BBox2D();
        ~BBox2D();

        void xmin(
            int _xmin);
        int xmin() const;
        void ymin(
            int _ymin);
        int ymin() const;
        void xmax(
            int _xmax);
        int xmax() const;
        void ymax(
            int _ymax);
        int ymax() const;
    };

    class CameraParkingStopper
    {
    public:
        CameraParkingStopper();
        ~CameraParkingStopper();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void bbox2d(
                const ros2_interface::msg::BBox2D &_bbox2d);
        const ros2_interface::msg::BBox2D &bbox2d() const;
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



    class BBox2DPubSubType
    {
    public:
        typedef BBox2D type;

        BBox2DPubSubType();
        ~BBox2DPubSubType();
    };

    class CameraParkingStopperPubSubType
    {
    public:
        typedef CameraParkingStopper type;

        CameraParkingStopperPubSubType();
        ~CameraParkingStopperPubSubType();
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

}
}


%template(CameraParkingStopperDataWriter) FastddsDataWriter<ros2_interface::msg::CameraParkingStopperPubSubType>;
%template(CameraParkingStopperDataReader) FastddsDataReader<ros2_interface::msg::CameraParkingStopperPubSubType>;
