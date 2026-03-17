%module fastdds_camera_parking_info_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "CameraParkingStopperPubSubTypes.h"
#include "CameraParkingInfoPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "ImageKeyPointPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "BBox2DPubSubTypes.h"
#include "CameraParkingInfoListPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
 typedef unsigned int uint32_t;
 typedef int int16_t;



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
    %template(VectorCameraParkingInfo) vector<ros2_interface::msg::CameraParkingInfo>;
    %template(VectorCameraParkingStopper) vector<ros2_interface::msg::CameraParkingStopper>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
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

    class CameraParkingInfo
    {
    public:
        CameraParkingInfo();
        ~CameraParkingInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void parking_space_id(
            int _parking_space_id);
        int parking_space_id() const;
        void parking_type(
            int _parking_type);
        int parking_type() const;
        void is_parking_enable(
            bool _is_parking_enable);
        bool is_parking_enable() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void center_point_of_parking(
                const ros2_interface::msg::Point3D &_center_point_of_parking);
        const ros2_interface::msg::Point3D &center_point_of_parking() const;
        void theta(
            double _theta);
        double theta() const;
        void width(
            double _width);
        double width() const;
        void length(
            double _length);
        double length() const;
        void yaw_offset(
            double _yaw_offset);
        double yaw_offset() const;
        void parking_points_in_image(
            const std::vector<ros2_interface::msg::ImageKeyPoint> &_parking_points_in_image);
        const std::vector<ros2_interface::msg::ImageKeyPoint>& parking_points_in_image() const;
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

    class ImageKeyPoint
    {
    public:
        ImageKeyPoint();
        ~ImageKeyPoint();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void confidence(
            double _confidence);
        double confidence() const;
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

    class CameraParkingInfoList
    {
    public:
        CameraParkingInfoList();
        ~CameraParkingInfoList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void camera_parking(
            const std::vector<ros2_interface::msg::CameraParkingInfo> &_camera_parking);
        const std::vector<ros2_interface::msg::CameraParkingInfo>& camera_parking() const;
        void camera_parking_stoppers(
            const std::vector<ros2_interface::msg::CameraParkingStopper> &_camera_parking_stoppers);
        const std::vector<ros2_interface::msg::CameraParkingStopper>& camera_parking_stoppers() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class CameraParkingStopperPubSubType
    {
    public:
        typedef CameraParkingStopper type;

        CameraParkingStopperPubSubType();
        ~CameraParkingStopperPubSubType();
    };

    class CameraParkingInfoPubSubType
    {
    public:
        typedef CameraParkingInfo type;

        CameraParkingInfoPubSubType();
        ~CameraParkingInfoPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class ImageKeyPointPubSubType
    {
    public:
        typedef ImageKeyPoint type;

        ImageKeyPointPubSubType();
        ~ImageKeyPointPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class BBox2DPubSubType
    {
    public:
        typedef BBox2D type;

        BBox2DPubSubType();
        ~BBox2DPubSubType();
    };

    class CameraParkingInfoListPubSubType
    {
    public:
        typedef CameraParkingInfoList type;

        CameraParkingInfoListPubSubType();
        ~CameraParkingInfoListPubSubType();
    };

}
}


%template(CameraParkingInfoListDataWriter) FastddsDataWriter<ros2_interface::msg::CameraParkingInfoListPubSubType>;
%template(CameraParkingInfoListDataReader) FastddsDataReader<ros2_interface::msg::CameraParkingInfoListPubSubType>;
