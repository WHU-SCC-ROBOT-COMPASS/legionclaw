%module fastdds_obstacle_feature 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "ObstacleFeaturePubSubTypes.h"
#include "TrajectoryPointInPredictionPubSubTypes.h"
#include "PredictionTrajectoryPointPubSubTypes.h"
#include "TrajectoryInPredictionPubSubTypes.h"
#include "ObstaclePriorityPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
%}
%include "std_vector.i"
 typedef int int32_t;



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
    %template(VectorPredictionTrajectoryPoint) vector<ros2_interface::msg::PredictionTrajectoryPoint>;
    %template(VectorTrajectoryPointInPrediction) vector<ros2_interface::msg::TrajectoryPointInPrediction>;
    %template(VectorTrajectoryInPrediction) vector<ros2_interface::msg::TrajectoryInPrediction>;
    %template(VectorTrajectoryPointInPrediction) vector<ros2_interface::msg::TrajectoryPointInPrediction>;
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

    class ObstacleFeature
    {
    public:
        ObstacleFeature();
        ~ObstacleFeature();

        void obstacle_id(
            int _obstacle_id);
        int obstacle_id() const;
        void polygon_point(
                const ros2_interface::msg::Point3D &_polygon_point);
        const ros2_interface::msg::Point3D &polygon_point() const;
        void position(
                const ros2_interface::msg::Point3D &_position);
        const ros2_interface::msg::Point3D &position() const;
        void front_position(
                const ros2_interface::msg::Point3D &_front_position);
        const ros2_interface::msg::Point3D &front_position() const;
        void velocity(
                const ros2_interface::msg::Point3D &_velocity);
        const ros2_interface::msg::Point3D &velocity() const;
        void raw_velocity(
                const ros2_interface::msg::Point3D &_raw_velocity);
        const ros2_interface::msg::Point3D &raw_velocity() const;
        void acceleration(
                const ros2_interface::msg::Point3D &_acceleration);
        const ros2_interface::msg::Point3D &acceleration() const;
        void velocity_heading(
            double _velocity_heading);
        double velocity_heading() const;
        void speed(
            double _speed);
        double speed() const;
        void acc(
            double _acc);
        double acc() const;
        void theta(
            double _theta);
        double theta() const;
        void length(
            double _length);
        double length() const;
        void width(
            double _width);
        double width() const;
        void height(
            double _height);
        double height() const;
        void tracking_time(
            double _tracking_time);
        double tracking_time() const;
        void timestamp(
            double _timestamp);
        double timestamp() const;
        void t_position(
                const ros2_interface::msg::Point3D &_t_position);
        const ros2_interface::msg::Point3D &t_position() const;
        void t_velocity(
                const ros2_interface::msg::Point3D &_t_velocity);
        const ros2_interface::msg::Point3D &t_velocity() const;
        void t_velocity_heading(
            double _t_velocity_heading);
        double t_velocity_heading() const;
        void t_speed(
            double _t_speed);
        double t_speed() const;
        void t_acceleration(
                const ros2_interface::msg::Point3D &_t_acceleration);
        const ros2_interface::msg::Point3D &t_acceleration() const;
        void t_acc(
            double _t_acc);
        double t_acc() const;
        void is_still(
            bool _is_still);
        bool is_still() const;
        void type(
            int _type);
        int type() const;
        void label_update_time_delta(
            double _label_update_time_delta);
        double label_update_time_delta() const;
        void priority(
                const ros2_interface::msg::ObstaclePriority &_priority);
        const ros2_interface::msg::ObstaclePriority &priority() const;
        void is_near_junction(
            bool _is_near_junction);
        bool is_near_junction() const;
        void future_trajectory_points(
            const std::vector<ros2_interface::msg::PredictionTrajectoryPoint> &_future_trajectory_points);
        const std::vector<ros2_interface::msg::PredictionTrajectoryPoint>& future_trajectory_points() const;
        void short_term_predicted_trajectory_points(
            const std::vector<ros2_interface::msg::TrajectoryPointInPrediction> &_short_term_predicted_trajectory_points);
        const std::vector<ros2_interface::msg::TrajectoryPointInPrediction>& short_term_predicted_trajectory_points() const;
        void predicted_trajectory(
            const std::vector<ros2_interface::msg::TrajectoryInPrediction> &_predicted_trajectory);
        const std::vector<ros2_interface::msg::TrajectoryInPrediction>& predicted_trajectory() const;
        void adc_trajectory_point(
            const std::vector<ros2_interface::msg::TrajectoryPointInPrediction> &_adc_trajectory_point);
        const std::vector<ros2_interface::msg::TrajectoryPointInPrediction>& adc_trajectory_point() const;
    };

    class TrajectoryPointInPrediction
    {
    public:
        TrajectoryPointInPrediction();
        ~TrajectoryPointInPrediction();

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
    };

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

    class TrajectoryInPrediction
    {
    public:
        TrajectoryInPrediction();
        ~TrajectoryInPrediction();

        void probability(
            double _probability);
        double probability() const;
        void trajectory_points(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_trajectory_points);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& trajectory_points() const;
    };

    class ObstaclePriority
    {
    public:
        ObstaclePriority();
        ~ObstaclePriority();

        void priority(
            int _priority);
        int priority() const;
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



    class PathPointPubSubType
    {
    public:
        typedef PathPoint type;

        PathPointPubSubType();
        ~PathPointPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class ObstacleFeaturePubSubType
    {
    public:
        typedef ObstacleFeature type;

        ObstacleFeaturePubSubType();
        ~ObstacleFeaturePubSubType();
    };

    class TrajectoryPointInPredictionPubSubType
    {
    public:
        typedef TrajectoryPointInPrediction type;

        TrajectoryPointInPredictionPubSubType();
        ~TrajectoryPointInPredictionPubSubType();
    };

    class PredictionTrajectoryPointPubSubType
    {
    public:
        typedef PredictionTrajectoryPoint type;

        PredictionTrajectoryPointPubSubType();
        ~PredictionTrajectoryPointPubSubType();
    };

    class TrajectoryInPredictionPubSubType
    {
    public:
        typedef TrajectoryInPrediction type;

        TrajectoryInPredictionPubSubType();
        ~TrajectoryInPredictionPubSubType();
    };

    class ObstaclePriorityPubSubType
    {
    public:
        typedef ObstaclePriority type;

        ObstaclePriorityPubSubType();
        ~ObstaclePriorityPubSubType();
    };

    class TrajectoryPointPubSubType
    {
    public:
        typedef TrajectoryPoint type;

        TrajectoryPointPubSubType();
        ~TrajectoryPointPubSubType();
    };

}
}


%template(ObstacleFeatureDataWriter) FastddsDataWriter<ros2_interface::msg::ObstacleFeaturePubSubType>;
%template(ObstacleFeatureDataReader) FastddsDataReader<ros2_interface::msg::ObstacleFeaturePubSubType>;
