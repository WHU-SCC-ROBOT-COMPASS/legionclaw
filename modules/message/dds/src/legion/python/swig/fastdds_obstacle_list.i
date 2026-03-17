%module fastdds_obstacle_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point2DPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "SensorCalibratorPubSubTypes.h"
#include "ObstacleListPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "ImageKeyPointPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "BBox2DPubSubTypes.h"
#include "ObstaclePubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef int int16_t;
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
    %template(VectorObstacle) vector<ros2_interface::msg::Obstacle>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Point2D
    {
    public:
        Point2D();
        ~Point2D();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
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

    class ObstacleList
    {
    public:
        ObstacleList();
        ~ObstacleList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void obstacle(
            const std::vector<ros2_interface::msg::Obstacle> &_obstacle);
        const std::vector<ros2_interface::msg::Obstacle>& obstacle() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
        void change_origin_flag(
            int _change_origin_flag);
        int change_origin_flag() const;
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

    class ImageKeyPoint
    {
    public:
        ImageKeyPoint();
        ~ImageKeyPoint();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void confidence(
            double _confidence);
        double confidence() const;
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

    class BBox2D
    {
    public:
        BBox2D();
        ~BBox2D();

        void xmin(
            int _xmin);
        int xmin() const;
        void ymin(
            int _ymin);
        int ymin() const;
        void xmax(
            int _xmax);
        int xmax() const;
        void ymax(
            int _ymax);
        int ymax() const;
    };

    class Obstacle
    {
    public:
        Obstacle();
        ~Obstacle();

        void timestamp(
                const ros2_interface::msg::Time &_timestamp);
        const ros2_interface::msg::Time &timestamp() const;
        void id(
            int _id);
        int id() const;
        void existence_prob(
            double _existence_prob);
        double existence_prob() const;
        void create_time(
                const ros2_interface::msg::Time &_create_time);
        const ros2_interface::msg::Time &create_time() const;
        void last_updated_time(
                const ros2_interface::msg::Time &_last_updated_time);
        const ros2_interface::msg::Time &last_updated_time() const;
        void center_pos_vehicle(
                const ros2_interface::msg::Point3D &_center_pos_vehicle);
        const ros2_interface::msg::Point3D &center_pos_vehicle() const;
        void center_pos_abs(
                const ros2_interface::msg::Point3D &_center_pos_abs);
        const ros2_interface::msg::Point3D &center_pos_abs() const;
        void theta_vehicle(
            double _theta_vehicle);
        double theta_vehicle() const;
        void theta_abs(
            double _theta_abs);
        double theta_abs() const;
        void velocity_vehicle(
                const ros2_interface::msg::Point3D &_velocity_vehicle);
        const ros2_interface::msg::Point3D &velocity_vehicle() const;
        void velocity_abs(
                const ros2_interface::msg::Point3D &_velocity_abs);
        const ros2_interface::msg::Point3D &velocity_abs() const;
        void length(
            double _length);
        double length() const;
        void width(
            double _width);
        double width() const;
        void height(
            double _height);
        double height() const;
        void image_key_points(
            const std::vector<ros2_interface::msg::ImageKeyPoint> &_image_key_points);
        const std::vector<ros2_interface::msg::ImageKeyPoint>& image_key_points() const;
        void polygon_point_abs(
            const std::vector<ros2_interface::msg::Point3D> &_polygon_point_abs);
        const std::vector<ros2_interface::msg::Point3D>& polygon_point_abs() const;
        void polygon_point_vehicle(
            const std::vector<ros2_interface::msg::Point3D> &_polygon_point_vehicle);
        const std::vector<ros2_interface::msg::Point3D>& polygon_point_vehicle() const;
        void tracking_time(
            double _tracking_time);
        double tracking_time() const;
        void type(
            int _type);
        int type() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void confidence_type(
            int _confidence_type);
        int confidence_type() const;
        void drops(
            const std::vector<ros2_interface::msg::Point3D> &_drops);
        const std::vector<ros2_interface::msg::Point3D>& drops() const;
        void acceleration_vehicle(
                const ros2_interface::msg::Point3D &_acceleration_vehicle);
        const ros2_interface::msg::Point3D &acceleration_vehicle() const;
        void acceleration_abs(
                const ros2_interface::msg::Point3D &_acceleration_abs);
        const ros2_interface::msg::Point3D &acceleration_abs() const;
        void anchor_point_image(
                const ros2_interface::msg::Point2D &_anchor_point_image);
        const ros2_interface::msg::Point2D &anchor_point_image() const;
        void anchor_point_vehicle(
                const ros2_interface::msg::Point3D &_anchor_point_vehicle);
        const ros2_interface::msg::Point3D &anchor_point_vehicle() const;
        void anchor_point_abs(
                const ros2_interface::msg::Point3D &_anchor_point_abs);
        const ros2_interface::msg::Point3D &anchor_point_abs() const;
        void bbox2d(
                const ros2_interface::msg::BBox2D &_bbox2d);
        const ros2_interface::msg::BBox2D &bbox2d() const;
        void bbox2d_rear(
                const ros2_interface::msg::BBox2D &_bbox2d_rear);
        const ros2_interface::msg::BBox2D &bbox2d_rear() const;
        void sub_type(
            int _sub_type);
        int sub_type() const;
        void height_above_ground(
            double _height_above_ground);
        double height_above_ground() const;
        void position_abs_covariance(
            const std::vector<double> &_position_abs_covariance);
        const std::vector<double>& position_abs_covariance() const;
        void velocity_abs_covariance(
            const std::vector<double> &_velocity_abs_covariance);
        const std::vector<double>& velocity_abs_covariance() const;
        void acceleration_abs_covariance(
            const std::vector<double> &_acceleration_abs_covariance);
        const std::vector<double>& acceleration_abs_covariance() const;
        void theta_abs_covariance(
            double _theta_abs_covariance);
        double theta_abs_covariance() const;
        void position_vehicle_covariance(
            const std::vector<double> &_position_vehicle_covariance);
        const std::vector<double>& position_vehicle_covariance() const;
        void velocity_vehicle_covariance(
            const std::vector<double> &_velocity_vehicle_covariance);
        const std::vector<double>& velocity_vehicle_covariance() const;
        void acceleration_vehicle_covariance(
            const std::vector<double> &_acceleration_vehicle_covariance);
        const std::vector<double>& acceleration_vehicle_covariance() const;
        void theta_vehicle_covariance(
            double _theta_vehicle_covariance);
        double theta_vehicle_covariance() const;
        void sensor_calibrator(
                const ros2_interface::msg::SensorCalibrator &_sensor_calibrator);
        const ros2_interface::msg::SensorCalibrator &sensor_calibrator() const;
        void cipv_flag(
            int _cipv_flag);
        int cipv_flag() const;
        void lane_position(
            int _lane_position);
        int lane_position() const;
        void pihp_percentage(
            double _pihp_percentage);
        double pihp_percentage() const;
        void blinker_flag(
            int _blinker_flag);
        int blinker_flag() const;
        void fusion_type(
            int _fusion_type);
        int fusion_type() const;
    };



    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class SensorCalibratorPubSubType
    {
    public:
        typedef SensorCalibrator type;

        SensorCalibratorPubSubType();
        ~SensorCalibratorPubSubType();
    };

    class ObstacleListPubSubType
    {
    public:
        typedef ObstacleList type;

        ObstacleListPubSubType();
        ~ObstacleListPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class ImageKeyPointPubSubType
    {
    public:
        typedef ImageKeyPoint type;

        ImageKeyPointPubSubType();
        ~ImageKeyPointPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class BBox2DPubSubType
    {
    public:
        typedef BBox2D type;

        BBox2DPubSubType();
        ~BBox2DPubSubType();
    };

    class ObstaclePubSubType
    {
    public:
        typedef Obstacle type;

        ObstaclePubSubType();
        ~ObstaclePubSubType();
    };

}
}


%template(ObstacleListDataWriter) FastddsDataWriter<ros2_interface::msg::ObstacleListPubSubType>;
%template(ObstacleListDataReader) FastddsDataReader<ros2_interface::msg::ObstacleListPubSubType>;
