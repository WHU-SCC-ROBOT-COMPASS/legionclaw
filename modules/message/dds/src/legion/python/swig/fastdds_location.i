%module fastdds_location 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointLLHPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "PointENUPubSubTypes.h"
#include "LocationPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
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
    class PointLLH
    {
    public:
        PointLLH();
        ~PointLLH();

        void lon(
            double _lon);
        double lon() const;
        void lat(
            double _lat);
        double lat() const;
        void height(
            double _height);
        double height() const;
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

    class PointENU
    {
    public:
        PointENU();
        ~PointENU();

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

    class Location
    {
    public:
        Location();
        ~Location();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void position(
                const ros2_interface::msg::PointLLH &_position);
        const ros2_interface::msg::PointLLH &position() const;
        void pitch(
            double _pitch);
        double pitch() const;
        void roll(
            double _roll);
        double roll() const;
        void heading(
            double _heading);
        double heading() const;
        void linear_velocity(
                const ros2_interface::msg::Point3D &_linear_velocity);
        const ros2_interface::msg::Point3D &linear_velocity() const;
        void linear_acceleration(
                const ros2_interface::msg::Point3D &_linear_acceleration);
        const ros2_interface::msg::Point3D &linear_acceleration() const;
        void angular_velocity(
                const ros2_interface::msg::Point3D &_angular_velocity);
        const ros2_interface::msg::Point3D &angular_velocity() const;
        void rtk_flag(
            int _rtk_flag);
        int rtk_flag() const;
        void odom_type(
            int _odom_type);
        int odom_type() const;
        void auxiliary_type(
            int _auxiliary_type);
        int auxiliary_type() const;
        void location_valid_flag(
            int _location_valid_flag);
        int location_valid_flag() const;
        void origin_lat(
            double _origin_lat);
        double origin_lat() const;
        void origin_lon(
            double _origin_lon);
        double origin_lon() const;
        void utm_position(
                const ros2_interface::msg::PointENU &_utm_position);
        const ros2_interface::msg::PointENU &utm_position() const;
        void change_origin_flag(
            int _change_origin_flag);
        int change_origin_flag() const;
        void utm_position_next(
                const ros2_interface::msg::PointENU &_utm_position_next);
        const ros2_interface::msg::PointENU &utm_position_next() const;
        void position_std_dev(
                const ros2_interface::msg::Point3D &_position_std_dev);
        const ros2_interface::msg::Point3D &position_std_dev() const;
        void orientation_std_dev(
                const ros2_interface::msg::Point3D &_orientation_std_dev);
        const ros2_interface::msg::Point3D &orientation_std_dev() const;
        void linear_velocity_std_dev(
                const ros2_interface::msg::Point3D &_linear_velocity_std_dev);
        const ros2_interface::msg::Point3D &linear_velocity_std_dev() const;
        void linear_acceleration_std_dev(
                const ros2_interface::msg::Point3D &_linear_acceleration_std_dev);
        const ros2_interface::msg::Point3D &linear_acceleration_std_dev() const;
        void angular_velocity_std_dev(
                const ros2_interface::msg::Point3D &_angular_velocity_std_dev);
        const ros2_interface::msg::Point3D &angular_velocity_std_dev() const;
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



    class PointLLHPubSubType
    {
    public:
        typedef PointLLH type;

        PointLLHPubSubType();
        ~PointLLHPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class PointENUPubSubType
    {
    public:
        typedef PointENU type;

        PointENUPubSubType();
        ~PointENUPubSubType();
    };

    class LocationPubSubType
    {
    public:
        typedef Location type;

        LocationPubSubType();
        ~LocationPubSubType();
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


%template(LocationDataWriter) FastddsDataWriter<ros2_interface::msg::LocationPubSubType>;
%template(LocationDataReader) FastddsDataReader<ros2_interface::msg::LocationPubSubType>;
