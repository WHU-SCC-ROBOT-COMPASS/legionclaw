%module fastdds_trajectory_array 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "TrajectoryPubSubTypes.h"
#include "PathPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "TrajectoryArrayPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;



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
    %template(VectorPath) vector<ros2_interface::msg::Path>;
    %template(VectorPath) vector<ros2_interface::msg::Path>;
    %template(VectorTrajectory) vector<ros2_interface::msg::Trajectory>;
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

    class Trajectory
    {
    public:
        Trajectory();
        ~Trajectory();

        void name(
           const std::string &_name);
        std::string name() const;
        void trajectory_points(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_trajectory_points);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& trajectory_points() const;
    };

    class Path
    {
    public:
        Path();
        ~Path();

        void name(
           const std::string &_name);
        std::string name() const;
        void path_points(
            const std::vector<ros2_interface::msg::PathPoint> &_path_points);
        const std::vector<ros2_interface::msg::PathPoint>& path_points() const;
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

    class TrajectoryArray
    {
    public:
        TrajectoryArray();
        ~TrajectoryArray();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void spline_s(
            const std::vector<ros2_interface::msg::Path> &_spline_s);
        const std::vector<ros2_interface::msg::Path>& spline_s() const;
        void qp_smooth(
            const std::vector<ros2_interface::msg::Path> &_qp_smooth);
        const std::vector<ros2_interface::msg::Path>& qp_smooth() const;
        void trajectory_list(
            const std::vector<ros2_interface::msg::Trajectory> &_trajectory_list);
        const std::vector<ros2_interface::msg::Trajectory>& trajectory_list() const;
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

    class TrajectoryPubSubType
    {
    public:
        typedef Trajectory type;

        TrajectoryPubSubType();
        ~TrajectoryPubSubType();
    };

    class PathPubSubType
    {
    public:
        typedef Path type;

        PathPubSubType();
        ~PathPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class TrajectoryArrayPubSubType
    {
    public:
        typedef TrajectoryArray type;

        TrajectoryArrayPubSubType();
        ~TrajectoryArrayPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
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


%template(TrajectoryArrayDataWriter) FastddsDataWriter<ros2_interface::msg::TrajectoryArrayPubSubType>;
%template(TrajectoryArrayDataReader) FastddsDataReader<ros2_interface::msg::TrajectoryArrayPubSubType>;
