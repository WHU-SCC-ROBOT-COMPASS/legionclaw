%module fastdds_odometry 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "QuaternionPubSubTypes.h"
#include "PointENUPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "OdometryPubSubTypes.h"
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

    class Odometry
    {
    public:
        Odometry();
        ~Odometry();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void position(
                const ros2_interface::msg::PointENU &_position);
        const ros2_interface::msg::PointENU &position() const;
        void orientation(
                const ros2_interface::msg::Quaternion &_orientation);
        const ros2_interface::msg::Quaternion &orientation() const;
        void covariance(
            const std::vector<double> &_covariance);
        const std::vector<double>& covariance() const;
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

    class OdometryPubSubType
    {
    public:
        typedef Odometry type;

        OdometryPubSubType();
        ~OdometryPubSubType();
    };

}
}


%template(OdometryDataWriter) FastddsDataWriter<ros2_interface::msg::OdometryPubSubType>;
%template(OdometryDataReader) FastddsDataReader<ros2_interface::msg::OdometryPubSubType>;
