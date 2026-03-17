%module fastdds_perception_obstacle 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "PerceptionObstaclePubSubTypes.h"
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
    %template(VectorPoint3D) vector<ros2_interface::msg::Point3D>;
    %template(VectorPoint3D) vector<ros2_interface::msg::Point3D>;
    %template(Vectordouble) vector<double>;
    %template(Vectordouble) vector<double>;
    %template(Vectordouble) vector<double>;
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



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class PerceptionObstaclePubSubType
    {
    public:
        typedef PerceptionObstacle type;

        PerceptionObstaclePubSubType();
        ~PerceptionObstaclePubSubType();
    };

}
}


%template(PerceptionObstacleDataWriter) FastddsDataWriter<ros2_interface::msg::PerceptionObstaclePubSubType>;
%template(PerceptionObstacleDataReader) FastddsDataReader<ros2_interface::msg::PerceptionObstaclePubSubType>;
