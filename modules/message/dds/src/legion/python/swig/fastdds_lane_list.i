%module fastdds_lane_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point2DPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "LaneLineCubicCurvePubSubTypes.h"
#include "SensorCalibratorPubSubTypes.h"
#include "RoadMarkPubSubTypes.h"
#include "HolisticPathPredictionPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "EndPointsPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "LaneListPubSubTypes.h"
#include "LaneLinePubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef int int8_t;
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
    %template(VectorLaneLine) vector<ros2_interface::msg::LaneLine>;
    %template(VectorRoadMark) vector<ros2_interface::msg::RoadMark>;
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

    class LaneLineCubicCurve
    {
    public:
        LaneLineCubicCurve();
        ~LaneLineCubicCurve();

        void start_x(
            double _start_x);
        double start_x() const;
        void end_x(
            double _end_x);
        double end_x() const;
        void a(
            double _a);
        double a() const;
        void b(
            double _b);
        double b() const;
        void c(
            double _c);
        double c() const;
        void d(
            double _d);
        double d() const;
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

    class RoadMark
    {
    public:
        RoadMark();
        ~RoadMark();

        void longitude_dist(
            double _longitude_dist);
        double longitude_dist() const;
        void lateral_dist(
            double _lateral_dist);
        double lateral_dist() const;
        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void type(
            int _type);
        int type() const;
    };

    class HolisticPathPrediction
    {
    public:
        HolisticPathPrediction();
        ~HolisticPathPrediction();

        void hpp(
                const ros2_interface::msg::LaneLineCubicCurve &_hpp);
        const ros2_interface::msg::LaneLineCubicCurve &hpp() const;
        void planning_source(
            int _planning_source);
        int planning_source() const;
        void ego_lane_width(
            double _ego_lane_width);
        double ego_lane_width() const;
        void confidence(
            double _confidence);
        double confidence() const;
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

    class EndPoints
    {
    public:
        EndPoints();
        ~EndPoints();

        void start(
                const ros2_interface::msg::Point2D &_start);
        const ros2_interface::msg::Point2D &start() const;
        void end(
                const ros2_interface::msg::Point2D &_end);
        const ros2_interface::msg::Point2D &end() const;
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

    class LaneList
    {
    public:
        LaneList();
        ~LaneList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void sensor_status(
            int _sensor_status);
        int sensor_status() const;
        void change_origin_flag(
            int _change_origin_flag);
        int change_origin_flag() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
        void sensor_calibrator(
                const ros2_interface::msg::SensorCalibrator &_sensor_calibrator);
        const ros2_interface::msg::SensorCalibrator &sensor_calibrator() const;
        void camera_laneline(
            const std::vector<ros2_interface::msg::LaneLine> &_camera_laneline);
        const std::vector<ros2_interface::msg::LaneLine>& camera_laneline() const;
        void hpp(
                const ros2_interface::msg::HolisticPathPrediction &_hpp);
        const ros2_interface::msg::HolisticPathPrediction &hpp() const;
        void road_marks(
            const std::vector<ros2_interface::msg::RoadMark> &_road_marks);
        const std::vector<ros2_interface::msg::RoadMark>& road_marks() const;
    };

    class LaneLine
    {
    public:
        LaneLine();
        ~LaneLine();

        void lane_type(
            int _lane_type);
        int lane_type() const;
        void lane_color(
            int _lane_color);
        int lane_color() const;
        void pos_type(
            int _pos_type);
        int pos_type() const;
        void curve_vehicle(
                const ros2_interface::msg::LaneLineCubicCurve &_curve_vehicle);
        const ros2_interface::msg::LaneLineCubicCurve &curve_vehicle() const;
        void curve_image(
                const ros2_interface::msg::LaneLineCubicCurve &_curve_image);
        const ros2_interface::msg::LaneLineCubicCurve &curve_image() const;
        void curve_abs(
                const ros2_interface::msg::LaneLineCubicCurve &_curve_abs);
        const ros2_interface::msg::LaneLineCubicCurve &curve_abs() const;
        void pts_vehicle(
            const std::vector<ros2_interface::msg::Point3D> &_pts_vehicle);
        const std::vector<ros2_interface::msg::Point3D>& pts_vehicle() const;
        void pts_image(
            const std::vector<ros2_interface::msg::Point2D> &_pts_image);
        const std::vector<ros2_interface::msg::Point2D>& pts_image() const;
        void pts_abs(
            const std::vector<ros2_interface::msg::Point3D> &_pts_abs);
        const std::vector<ros2_interface::msg::Point3D>& pts_abs() const;
        void image_end_point(
                const ros2_interface::msg::EndPoints &_image_end_point);
        const ros2_interface::msg::EndPoints &image_end_point() const;
        void pts_key(
            const std::vector<ros2_interface::msg::Point2D> &_pts_key);
        const std::vector<ros2_interface::msg::Point2D>& pts_key() const;
        void hd_lane_id(
            int _hd_lane_id);
        int hd_lane_id() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void lane_quality(
            int _lane_quality);
        int lane_quality() const;
        void fused_lane_type(
            int _fused_lane_type);
        int fused_lane_type() const;
        void homography_mat(
            const std::vector<double> &_homography_mat);
        const std::vector<double>& homography_mat() const;
        void homography_mat_inv(
            const std::vector<double> &_homography_mat_inv);
        const std::vector<double>& homography_mat_inv() const;
        void lane_coordinate_type(
            int _lane_coordinate_type);
        int lane_coordinate_type() const;
        void use_type(
            int _use_type);
        int use_type() const;
        void create_time(
                const ros2_interface::msg::Time &_create_time);
        const ros2_interface::msg::Time &create_time() const;
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

    class LaneLineCubicCurvePubSubType
    {
    public:
        typedef LaneLineCubicCurve type;

        LaneLineCubicCurvePubSubType();
        ~LaneLineCubicCurvePubSubType();
    };

    class SensorCalibratorPubSubType
    {
    public:
        typedef SensorCalibrator type;

        SensorCalibratorPubSubType();
        ~SensorCalibratorPubSubType();
    };

    class RoadMarkPubSubType
    {
    public:
        typedef RoadMark type;

        RoadMarkPubSubType();
        ~RoadMarkPubSubType();
    };

    class HolisticPathPredictionPubSubType
    {
    public:
        typedef HolisticPathPrediction type;

        HolisticPathPredictionPubSubType();
        ~HolisticPathPredictionPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class EndPointsPubSubType
    {
    public:
        typedef EndPoints type;

        EndPointsPubSubType();
        ~EndPointsPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class LaneListPubSubType
    {
    public:
        typedef LaneList type;

        LaneListPubSubType();
        ~LaneListPubSubType();
    };

    class LaneLinePubSubType
    {
    public:
        typedef LaneLine type;

        LaneLinePubSubType();
        ~LaneLinePubSubType();
    };

}
}


%template(LaneListDataWriter) FastddsDataWriter<ros2_interface::msg::LaneListPubSubType>;
%template(LaneListDataReader) FastddsDataReader<ros2_interface::msg::LaneListPubSubType>;
