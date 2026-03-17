%module fastdds_uncertainty 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "UncertaintyPubSubTypes.h"
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

    class Uncertainty
    {
    public:
        Uncertainty();
        ~Uncertainty();

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



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class UncertaintyPubSubType
    {
    public:
        typedef Uncertainty type;

        UncertaintyPubSubType();
        ~UncertaintyPubSubType();
    };

}
}


%template(UncertaintyDataWriter) FastddsDataWriter<ros2_interface::msg::UncertaintyPubSubType>;
%template(UncertaintyDataReader) FastddsDataReader<ros2_interface::msg::UncertaintyPubSubType>;
