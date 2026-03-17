%module fastdds_parking_out_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "ParkingOutInfoPubSubTypes.h"
%}
%include "std_string.i"
 typedef unsigned int uint32_t;
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

    class ParkingOutInfo
    {
    public:
        ParkingOutInfo();
        ~ParkingOutInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void parking_out_id(
            int _parking_out_id);
        int parking_out_id() const;
        void parking_direction_type(
            int _parking_direction_type);
        int parking_direction_type() const;
        void is_parking_out_enable(
            bool _is_parking_out_enable);
        bool is_parking_out_enable() const;
        void parking_out_point(
                const ros2_interface::msg::Point3D &_parking_out_point);
        const ros2_interface::msg::Point3D &parking_out_point() const;
        void theta(
            double _theta);
        double theta() const;
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

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class ParkingOutInfoPubSubType
    {
    public:
        typedef ParkingOutInfo type;

        ParkingOutInfoPubSubType();
        ~ParkingOutInfoPubSubType();
    };

}
}


%template(ParkingOutInfoDataWriter) FastddsDataWriter<ros2_interface::msg::ParkingOutInfoPubSubType>;
%template(ParkingOutInfoDataReader) FastddsDataReader<ros2_interface::msg::ParkingOutInfoPubSubType>;
