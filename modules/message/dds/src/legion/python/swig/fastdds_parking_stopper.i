%module fastdds_parking_stopper 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ParkingStopperPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
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
    %template(VectorPoint3D) vector<ros2_interface::msg::Point3D>;
    %template(VectorPoint3D) vector<ros2_interface::msg::Point3D>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class ParkingStopper
    {
    public:
        ParkingStopper();
        ~ParkingStopper();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void center_point_vehicle(
                const ros2_interface::msg::Point3D &_center_point_vehicle);
        const ros2_interface::msg::Point3D &center_point_vehicle() const;
        void center_point_abs(
                const ros2_interface::msg::Point3D &_center_point_abs);
        const ros2_interface::msg::Point3D &center_point_abs() const;
        void stopper_points_vehicle(
            const std::vector<ros2_interface::msg::Point3D> &_stopper_points_vehicle);
        const std::vector<ros2_interface::msg::Point3D>& stopper_points_vehicle() const;
        void stopper_points_abs(
            const std::vector<ros2_interface::msg::Point3D> &_stopper_points_abs);
        const std::vector<ros2_interface::msg::Point3D>& stopper_points_abs() const;
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



    class ParkingStopperPubSubType
    {
    public:
        typedef ParkingStopper type;

        ParkingStopperPubSubType();
        ~ParkingStopperPubSubType();
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

}
}


%template(ParkingStopperDataWriter) FastddsDataWriter<ros2_interface::msg::ParkingStopperPubSubType>;
%template(ParkingStopperDataReader) FastddsDataReader<ros2_interface::msg::ParkingStopperPubSubType>;
