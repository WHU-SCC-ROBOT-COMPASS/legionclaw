%module fastdds_sensor_calibrator 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SensorCalibratorPubSubTypes.h"
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
    class SensorCalibrator
    {
    public:
        SensorCalibrator();
        ~SensorCalibrator();

        void pose(
                const ros2_interface::msg::Point3D &_pose);
        const ros2_interface::msg::Point3D &pose() const;
        void angle(
                const ros2_interface::msg::Point3D &_angle);
        const ros2_interface::msg::Point3D &angle() const;
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



    class SensorCalibratorPubSubType
    {
    public:
        typedef SensorCalibrator type;

        SensorCalibratorPubSubType();
        ~SensorCalibratorPubSubType();
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


%template(SensorCalibratorDataWriter) FastddsDataWriter<ros2_interface::msg::SensorCalibratorPubSubType>;
%template(SensorCalibratorDataReader) FastddsDataReader<ros2_interface::msg::SensorCalibratorPubSubType>;
