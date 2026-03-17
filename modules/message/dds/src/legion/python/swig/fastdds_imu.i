%module fastdds_imu 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImuPubSubTypes.h"
#include "QuaternionPubSubTypes.h"
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
    %template(Vectordouble) vector<double>;
    %template(Vectordouble) vector<double>;
    %template(Vectordouble) vector<double>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Imu
    {
    public:
        Imu();
        ~Imu();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
        void orientation(
                const ros2_interface::msg::Quaternion &_orientation);
        const ros2_interface::msg::Quaternion &orientation() const;
        void orientation_covariance(
            const std::vector<double> &_orientation_covariance);
        const std::vector<double>& orientation_covariance() const;
        void angular_velocity(
                const ros2_interface::msg::Point3D &_angular_velocity);
        const ros2_interface::msg::Point3D &angular_velocity() const;
        void angular_velocity_covariance(
            const std::vector<double> &_angular_velocity_covariance);
        const std::vector<double>& angular_velocity_covariance() const;
        void linear_acceleration(
                const ros2_interface::msg::Point3D &_linear_acceleration);
        const ros2_interface::msg::Point3D &linear_acceleration() const;
        void linear_acceleration_covariance(
            const std::vector<double> &_linear_acceleration_covariance);
        const std::vector<double>& linear_acceleration_covariance() const;
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



    class ImuPubSubType
    {
    public:
        typedef Imu type;

        ImuPubSubType();
        ~ImuPubSubType();
    };

    class QuaternionPubSubType
    {
    public:
        typedef Quaternion type;

        QuaternionPubSubType();
        ~QuaternionPubSubType();
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


%template(ImuDataWriter) FastddsDataWriter<ros2_interface::msg::ImuPubSubType>;
%template(ImuDataReader) FastddsDataReader<ros2_interface::msg::ImuPubSubType>;
