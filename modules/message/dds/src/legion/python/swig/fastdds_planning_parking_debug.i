%module fastdds_planning_parking_debug 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PathPointPubSubTypes.h"
#include "TrajectoryPubSubTypes.h"
#include "Polygon2DPubSubTypes.h"
#include "Point2dListPubSubTypes.h"
#include "Point2DPubSubTypes.h"
#include "PlanningParkingDebugPubSubTypes.h"
#include "TrajectoryPointPubSubTypes.h"
%}
%include "std_string.i"
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
    %template(VectorPolygon2D) vector<ros2_interface::msg::Polygon2D>;
    %template(VectorPolygon2D) vector<ros2_interface::msg::Polygon2D>;
    %template(VectorPathPoint) vector<ros2_interface::msg::PathPoint>;
    %template(VectorPoint2dList) vector<ros2_interface::msg::Point2dList>;
    %template(VectorTrajectoryPoint) vector<ros2_interface::msg::TrajectoryPoint>;
    %template(VectorTrajectoryPoint) vector<ros2_interface::msg::TrajectoryPoint>;
    %template(VectorTrajectoryPoint) vector<ros2_interface::msg::TrajectoryPoint>;
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

    class Polygon2D
    {
    public:
        Polygon2D();
        ~Polygon2D();

        void coordinate_system(
            int _coordinate_system);
        int coordinate_system() const;
        void points(
            const std::vector<ros2_interface::msg::Point2D> &_points);
        const std::vector<ros2_interface::msg::Point2D>& points() const;
    };

    class Point2dList
    {
    public:
        Point2dList();
        ~Point2dList();

        void point2d_list(
            const std::vector<ros2_interface::msg::Point2D> &_point2d_list);
        const std::vector<ros2_interface::msg::Point2D>& point2d_list() const;
    };

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

    class PlanningParkingDebug
    {
    public:
        PlanningParkingDebug();
        ~PlanningParkingDebug();

        void vehicle_preiew_polygon(
            const std::vector<ros2_interface::msg::Polygon2D> &_vehicle_preiew_polygon);
        const std::vector<ros2_interface::msg::Polygon2D>& vehicle_preiew_polygon() const;
        void obstacles_polygon(
            const std::vector<ros2_interface::msg::Polygon2D> &_obstacles_polygon);
        const std::vector<ros2_interface::msg::Polygon2D>& obstacles_polygon() const;
        void path_points(
            const std::vector<ros2_interface::msg::PathPoint> &_path_points);
        const std::vector<ros2_interface::msg::PathPoint>& path_points() const;
        void lat_error(
            double _lat_error);
        double lat_error() const;
        void lon_error(
            double _lon_error);
        double lon_error() const;
        void yaw_error(
            double _yaw_error);
        double yaw_error() const;
        void obstacles_vec(
            const std::vector<ros2_interface::msg::Point2dList> &_obstacles_vec);
        const std::vector<ros2_interface::msg::Point2dList>& obstacles_vec() const;
        void warm_start_traj(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_warm_start_traj);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& warm_start_traj() const;
        void smoothed_traj_stage1(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_smoothed_traj_stage1);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& smoothed_traj_stage1() const;
        void smoothed_traj_stage2(
            const std::vector<ros2_interface::msg::TrajectoryPoint> &_smoothed_traj_stage2);
        const std::vector<ros2_interface::msg::TrajectoryPoint>& smoothed_traj_stage2() const;
        void reference_line(
                const ros2_interface::msg::Trajectory &_reference_line);
        const ros2_interface::msg::Trajectory &reference_line() const;
        void trajectory_array(
            const std::vector<ros2_interface::msg::Trajectory> &_trajectory_array);
        const std::vector<ros2_interface::msg::Trajectory>& trajectory_array() const;
        void optimal_coarse_trajectory(
                const ros2_interface::msg::Trajectory &_optimal_coarse_trajectory);
        const ros2_interface::msg::Trajectory &optimal_coarse_trajectory() const;
        void optimal_smooth_trajectory(
                const ros2_interface::msg::Trajectory &_optimal_smooth_trajectory);
        const ros2_interface::msg::Trajectory &optimal_smooth_trajectory() const;
        void hybrid_a_star_map_time(
            double _hybrid_a_star_map_time);
        double hybrid_a_star_map_time() const;
        void hybrid_a_star_heuristic_time(
            double _hybrid_a_star_heuristic_time);
        double hybrid_a_star_heuristic_time() const;
        void hybrid_a_star_rs_time(
            double _hybrid_a_star_rs_time);
        double hybrid_a_star_rs_time() const;
        void hybrid_a_star_total_time(
            double _hybrid_a_star_total_time);
        double hybrid_a_star_total_time() const;
        void ias_collision_avoidance_time(
            double _ias_collision_avoidance_time);
        double ias_collision_avoidance_time() const;
        void ias_path_smooth_time(
            double _ias_path_smooth_time);
        double ias_path_smooth_time() const;
        void ias_speed_smooth_time(
            double _ias_speed_smooth_time);
        double ias_speed_smooth_time() const;
        void ias_total_time(
            double _ias_total_time);
        double ias_total_time() const;
        void samping_trajectory_time(
            double _samping_trajectory_time);
        double samping_trajectory_time() const;
        void is_replan(
            bool _is_replan);
        bool is_replan() const;
        void replan_reason(
           const std::string &_replan_reason);
        std::string replan_reason() const;
        void replan_time(
            double _replan_time);
        double replan_time() const;
        void replan_num(
            int _replan_num);
        int replan_num() const;
        void optimizer_thread_counter(
            int _optimizer_thread_counter);
        int optimizer_thread_counter() const;
        void replan_by_context_update_counter(
            int _replan_by_context_update_counter);
        int replan_by_context_update_counter() const;
        void replan_by_large_error_counter(
            int _replan_by_large_error_counter);
        int replan_by_large_error_counter() const;
        void parking_type(
            int _parking_type);
        int parking_type() const;
        void moves_counter(
            int _moves_counter);
        int moves_counter() const;
        void remain_distance(
            double _remain_distance);
        double remain_distance() const;
        void distance_to_leader_obj(
            double _distance_to_leader_obj);
        double distance_to_leader_obj() const;
        void state(
            int _state);
        int state() const;
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

    class Polygon2DPubSubType
    {
    public:
        typedef Polygon2D type;

        Polygon2DPubSubType();
        ~Polygon2DPubSubType();
    };

    class Point2dListPubSubType
    {
    public:
        typedef Point2dList type;

        Point2dListPubSubType();
        ~Point2dListPubSubType();
    };

    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

    class PlanningParkingDebugPubSubType
    {
    public:
        typedef PlanningParkingDebug type;

        PlanningParkingDebugPubSubType();
        ~PlanningParkingDebugPubSubType();
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


%template(PlanningParkingDebugDataWriter) FastddsDataWriter<ros2_interface::msg::PlanningParkingDebugPubSubType>;
%template(PlanningParkingDebugDataReader) FastddsDataReader<ros2_interface::msg::PlanningParkingDebugPubSubType>;
