%module fastdds_vehicle_state 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "QuaternionPubSubTypes.h"
#include "PointENUPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "VehicleStatePubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "PosePubSubTypes.h"
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
    class Quaternion
    {
    public:
        Quaternion();
        ~Quaternion();

        void qx(
            double _qx);
        double qx() const;
        void qy(
            double _qy);
        double qy() const;
        void qz(
            double _qz);
        double qz() const;
        void qw(
            double _qw);
        double qw() const;
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

    class VehicleState
    {
    public:
        VehicleState();
        ~VehicleState();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void timestamp(
            double _timestamp);
        double timestamp() const;
        void roll(
            double _roll);
        double roll() const;
        void pitch(
            double _pitch);
        double pitch() const;
        void yaw(
            double _yaw);
        double yaw() const;
        void heading(
            double _heading);
        double heading() const;
        void kappa(
            double _kappa);
        double kappa() const;
        void linear_velocity(
            double _linear_velocity);
        double linear_velocity() const;
        void angular_velocity(
            double _angular_velocity);
        double angular_velocity() const;
        void linear_acceleration(
            double _linear_acceleration);
        double linear_acceleration() const;
        void gear(
            int _gear);
        int gear() const;
        void driving_mode(
            int _driving_mode);
        int driving_mode() const;
        void pose(
                const ros2_interface::msg::Pose &_pose);
        const ros2_interface::msg::Pose &pose() const;
        void front_steering_value(
            double _front_steering_value);
        double front_steering_value() const;
        void rear_steering_value(
            double _rear_steering_value);
        double rear_steering_value() const;
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

    class Pose
    {
    public:
        Pose();
        ~Pose();

        void position(
                const ros2_interface::msg::PointENU &_position);
        const ros2_interface::msg::PointENU &position() const;
        void orientation(
                const ros2_interface::msg::Quaternion &_orientation);
        const ros2_interface::msg::Quaternion &orientation() const;
        void linear_velocity(
                const ros2_interface::msg::Point3D &_linear_velocity);
        const ros2_interface::msg::Point3D &linear_velocity() const;
        void linear_acceleration(
                const ros2_interface::msg::Point3D &_linear_acceleration);
        const ros2_interface::msg::Point3D &linear_acceleration() const;
        void angular_velocity(
                const ros2_interface::msg::Point3D &_angular_velocity);
        const ros2_interface::msg::Point3D &angular_velocity() const;
        void heading(
            double _heading);
        double heading() const;
        void linear_acceleration_vrf(
                const ros2_interface::msg::Point3D &_linear_acceleration_vrf);
        const ros2_interface::msg::Point3D &linear_acceleration_vrf() const;
        void angular_velocity_vrf(
                const ros2_interface::msg::Point3D &_angular_velocity_vrf);
        const ros2_interface::msg::Point3D &angular_velocity_vrf() const;
        void euler_angles(
                const ros2_interface::msg::Point3D &_euler_angles);
        const ros2_interface::msg::Point3D &euler_angles() const;
    };



    class QuaternionPubSubType
    {
    public:
        typedef Quaternion type;

        QuaternionPubSubType();
        ~QuaternionPubSubType();
    };

    class PointENUPubSubType
    {
    public:
        typedef PointENU type;

        PointENUPubSubType();
        ~PointENUPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class VehicleStatePubSubType
    {
    public:
        typedef VehicleState type;

        VehicleStatePubSubType();
        ~VehicleStatePubSubType();
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

    class PosePubSubType
    {
    public:
        typedef Pose type;

        PosePubSubType();
        ~PosePubSubType();
    };

}
}


%template(VehicleStateDataWriter) FastddsDataWriter<ros2_interface::msg::VehicleStatePubSubType>;
%template(VehicleStateDataReader) FastddsDataReader<ros2_interface::msg::VehicleStatePubSubType>;
