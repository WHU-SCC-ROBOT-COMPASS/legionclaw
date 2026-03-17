%module fastdds_vehicle_motion_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
#include "VehicleMotionPointPubSubTypes.h"
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
    class PathPoint
    {
    public:
        PathPoint();
        ~PathPoint();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void theta(
            double _theta);
        double theta() const;
        void kappa(
            double _kappa);
        double kappa() const;
        void s(
            double _s);
        double s() const;
        void dkappa(
            double _dkappa);
        double dkappa() const;
        void ddkappa(
            double _ddkappa);
        double ddkappa() const;
        void lane_id(
            double _lane_id);
        double lane_id() const;
        void x_derivative(
            double _x_derivative);
        double x_derivative() const;
        void y_derivative(
            double _y_derivative);
        double y_derivative() const;
    };

    class TrajectoryPoint
    {
    public:
        TrajectoryPoint();
        ~TrajectoryPoint();

        void path_point(
                const ros2_interface::msg::PathPoint &_path_point);
        const ros2_interface::msg::PathPoint &path_point() const;
        void v(
            double _v);
        double v() const;
        void a(
            double _a);
        double a() const;
        void relative_time(
            double _relative_time);
        double relative_time() const;
        void da(
            double _da);
        double da() const;
        void is_steer_valid(
            bool _is_steer_valid);
        bool is_steer_valid() const;
        void front_steer(
            double _front_steer);
        double front_steer() const;
        void rear_steer(
            double _rear_steer);
        double rear_steer() const;
        void gear(
            int _gear);
        int gear() const;
    };

    class VehicleMotionPoint
    {
    public:
        VehicleMotionPoint();
        ~VehicleMotionPoint();

        void trajectory_point(
                const ros2_interface::msg::TrajectoryPoint &_trajectory_point);
        const ros2_interface::msg::TrajectoryPoint &trajectory_point() const;
        void steer(
            double _steer);
        double steer() const;
    };



    class PathPointPubSubType
    {
    public:
        typedef PathPoint type;

        PathPointPubSubType();
        ~PathPointPubSubType();
    };

    class TrajectoryPointPubSubType
    {
    public:
        typedef TrajectoryPoint type;

        TrajectoryPointPubSubType();
        ~TrajectoryPointPubSubType();
    };

    class VehicleMotionPointPubSubType
    {
    public:
        typedef VehicleMotionPoint type;

        VehicleMotionPointPubSubType();
        ~VehicleMotionPointPubSubType();
    };

}
}


%template(VehicleMotionPointDataWriter) FastddsDataWriter<ros2_interface::msg::VehicleMotionPointPubSubType>;
%template(VehicleMotionPointDataReader) FastddsDataReader<ros2_interface::msg::VehicleMotionPointPubSubType>;
