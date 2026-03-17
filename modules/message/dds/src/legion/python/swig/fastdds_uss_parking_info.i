%module fastdds_uss_parking_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "UssParkingInfoPubSubTypes.h"
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

    class UssParkingInfo
    {
    public:
        UssParkingInfo();
        ~UssParkingInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void uss_parking_type(
            int _uss_parking_type);
        int uss_parking_type() const;
        void parking_point0(
                const ros2_interface::msg::Point3D &_parking_point0);
        const ros2_interface::msg::Point3D &parking_point0() const;
        void parking_point1(
                const ros2_interface::msg::Point3D &_parking_point1);
        const ros2_interface::msg::Point3D &parking_point1() const;
        void parking_point2(
                const ros2_interface::msg::Point3D &_parking_point2);
        const ros2_interface::msg::Point3D &parking_point2() const;
        void parking_point3(
                const ros2_interface::msg::Point3D &_parking_point3);
        const ros2_interface::msg::Point3D &parking_point3() const;
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



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class UssParkingInfoPubSubType
    {
    public:
        typedef UssParkingInfo type;

        UssParkingInfoPubSubType();
        ~UssParkingInfoPubSubType();
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


%template(UssParkingInfoDataWriter) FastddsDataWriter<ros2_interface::msg::UssParkingInfoPubSubType>;
%template(UssParkingInfoDataReader) FastddsDataReader<ros2_interface::msg::UssParkingInfoPubSubType>;
