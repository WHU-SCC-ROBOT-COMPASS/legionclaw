%module fastdds_prediction_trajectory_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PredictionTrajectoryPointPubSubTypes.h"
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
    class PredictionTrajectoryPoint
    {
    public:
        PredictionTrajectoryPoint();
        ~PredictionTrajectoryPoint();

        void predition_path_point(
                const ros2_interface::msg::Point3D &_predition_path_point);
        const ros2_interface::msg::Point3D &predition_path_point() const;
        void timestamp(
            double _timestamp);
        double timestamp() const;
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



    class PredictionTrajectoryPointPubSubType
    {
    public:
        typedef PredictionTrajectoryPoint type;

        PredictionTrajectoryPointPubSubType();
        ~PredictionTrajectoryPointPubSubType();
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


%template(PredictionTrajectoryPointDataWriter) FastddsDataWriter<ros2_interface::msg::PredictionTrajectoryPointPubSubType>;
%template(PredictionTrajectoryPointDataReader) FastddsDataReader<ros2_interface::msg::PredictionTrajectoryPointPubSubType>;
