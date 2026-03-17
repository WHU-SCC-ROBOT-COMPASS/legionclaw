%module fastdds_prediction_obstacle 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "PredictionObstaclePubSubTypes.h"
#include "ObstacleIntentPubSubTypes.h"
#include "PerceptionObstaclePubSubTypes.h"
#include "TrajectoryInPredictionPubSubTypes.h"
#include "ObstacleInteractiveTagPubSubTypes.h"
#include "ObstaclePriorityPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
%}
%include "std_vector.i"
 typedef int int32_t;
 typedef unsigned int uint8_t;



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
    %template(VectorTrajectoryInPrediction) vector<ros2_interface::msg::TrajectoryInPrediction>;
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

    class PredictionObstacle
    {
    public:
        PredictionObstacle();
        ~PredictionObstacle();

        void perception_obstacle(
                const ros2_interface::msg::PerceptionObstacle &_perception_obstacle);
        const ros2_interface::msg::PerceptionObstacle &perception_obstacle() const;
        void timestamp(
            double _timestamp);
        double timestamp() const;
        void predicted_period(
            double _predicted_period);
        double predicted_period() const;
        void trajectory(
            const std::vector<ros2_interface::msg::TrajectoryInPrediction> &_trajectory);
        const std::vector<ros2_interface::msg::TrajectoryInPrediction>& trajectory() const;
        void intent(
                const ros2_interface::msg::ObstacleIntent &_intent);
        const ros2_interface::msg::ObstacleIntent &intent() const;
        void priority(
                const ros2_interface::msg::ObstaclePriority &_priority);
        const ros2_interface::msg::ObstaclePriority &priority() const;
        void interactive_tag(
                const ros2_interface::msg::ObstacleInteractiveTag &_interactive_tag);
        const ros2_interface::msg::ObstacleInteractiveTag &interactive_tag() const;
        void is_static(
            bool _is_static);
        bool is_static() const;
    };

    class ObstacleIntent
    {
    public:
        ObstacleIntent();
        ~ObstacleIntent();

        void type(
            int _type);
        int type() const;
    };

    class PerceptionObstacle
    {
    public:
        PerceptionObstacle();
        ~PerceptionObstacle();

        void id(
            int _id);
        int id() const;
        void position(
                const ros2_interface::msg::Point3D &_position);
        const ros2_interface::msg::Point3D &position() const;
        void theta(
            double _theta);
        double theta() const;
        void velocity(
                const ros2_interface::msg::Point3D &_velocity);
        const ros2_interface::msg::Point3D &velocity() const;
        void length(
            double _length);
        double length() const;
        void width(
            double _width);
        double width() const;
        void height(
            double _height);
        double height() const;
        void polygon_point(
            const std::vector<ros2_interface::msg::Point3D> &_polygon_point);
        const std::vector<ros2_interface::msg::Point3D>& polygon_point() const;
        void tracking_time(
            double _tracking_time);
        double tracking_time() const;
        void type(
            int _type);
        int type() const;
        void lane_position(
            int _lane_position);
        int lane_position() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void timestamp(
            double _timestamp);
        double timestamp() const;
        void confidence_type(
            int _confidence_type);
        int confidence_type() const;
        void drops(
                const ros2_interface::msg::Point3D &_drops);
        const ros2_interface::msg::Point3D &drops() const;
        void acceleration(
                const ros2_interface::msg::Point3D &_acceleration);
        const ros2_interface::msg::Point3D &acceleration() const;
        void anchor_point(
                const ros2_interface::msg::Point3D &_anchor_point);
        const ros2_interface::msg::Point3D &anchor_point() const;
        void bounding_box(
            const std::vector<ros2_interface::msg::Point3D> &_bounding_box);
        const std::vector<ros2_interface::msg::Point3D>& bounding_box() const;
        void sub_type(
            int _sub_type);
        int sub_type() const;
        void height_above_ground(
            double _height_above_ground);
        double height_above_ground() const;
        void position_covariance(
            const std::vector<double> &_position_covariance);
        const std::vector<double>& position_covariance() const;
        void velocity_covariance(
            const std::vector<double> &_velocity_covariance);
        const std::vector<double>& velocity_covariance() const;
        void acceleration_covariance(
            const std::vector<double> &_acceleration_covariance);
        const std::vector<double>& acceleration_covariance() const;
        void light_status(
            int _light_status);
        int light_status() const;
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

    class ObstacleInteractiveTag
    {
    public:
        ObstacleInteractiveTag();
        ~ObstacleInteractiveTag();

        void interactive_tag(
            int _interactive_tag);
        int interactive_tag() const;
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

    class PredictionObstaclePubSubType
    {
    public:
        typedef PredictionObstacle type;

        PredictionObstaclePubSubType();
        ~PredictionObstaclePubSubType();
    };

    class ObstacleIntentPubSubType
    {
    public:
        typedef ObstacleIntent type;

        ObstacleIntentPubSubType();
        ~ObstacleIntentPubSubType();
    };

    class PerceptionObstaclePubSubType
    {
    public:
        typedef PerceptionObstacle type;

        PerceptionObstaclePubSubType();
        ~PerceptionObstaclePubSubType();
    };

    class TrajectoryInPredictionPubSubType
    {
    public:
        typedef TrajectoryInPrediction type;

        TrajectoryInPredictionPubSubType();
        ~TrajectoryInPredictionPubSubType();
    };

    class ObstacleInteractiveTagPubSubType
    {
    public:
        typedef ObstacleInteractiveTag type;

        ObstacleInteractiveTagPubSubType();
        ~ObstacleInteractiveTagPubSubType();
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


%template(PredictionObstacleDataWriter) FastddsDataWriter<ros2_interface::msg::PredictionObstaclePubSubType>;
%template(PredictionObstacleDataReader) FastddsDataReader<ros2_interface::msg::PredictionObstaclePubSubType>;
