%module fastdds_hmi_trajectory_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HMITrajectoryPointPubSubTypes.h"
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
    class HMITrajectoryPoint
    {
    public:
        HMITrajectoryPoint();
        ~HMITrajectoryPoint();

        void point(
                const ros2_interface::msg::Point3D &_point);
        const ros2_interface::msg::Point3D &point() const;
        void v(
            double _v);
        double v() const;
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



    class HMITrajectoryPointPubSubType
    {
    public:
        typedef HMITrajectoryPoint type;

        HMITrajectoryPointPubSubType();
        ~HMITrajectoryPointPubSubType();
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


%template(HMITrajectoryPointDataWriter) FastddsDataWriter<ros2_interface::msg::HMITrajectoryPointPubSubType>;
%template(HMITrajectoryPointDataReader) FastddsDataReader<ros2_interface::msg::HMITrajectoryPointPubSubType>;
