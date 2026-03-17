%module fastdds_pose 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PosePubSubTypes.h"
#include "QuaternionPubSubTypes.h"
#include "PointENUPubSubTypes.h"
#include "Point3DPubSubTypes.h"
%}



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



    class PosePubSubType
    {
    public:
        typedef Pose type;

        PosePubSubType();
        ~PosePubSubType();
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

}
}


%template(PoseDataWriter) FastddsDataWriter<ros2_interface::msg::PosePubSubType>;
%template(PoseDataReader) FastddsDataReader<ros2_interface::msg::PosePubSubType>;
