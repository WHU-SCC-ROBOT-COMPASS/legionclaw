%module fastdds_lane_info_extend 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "LanePointPubSubTypes.h"
#include "Point3DPubSubTypes.h"
#include "LaneInfoExtendPubSubTypes.h"
%}
%include "std_vector.i"
 typedef int int8_t;
 typedef int int64_t;



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
    %template(Vectorint) vector<int>;
    %template(Vectorint) vector<int>;
    %template(VectorLanePoint) vector<ros2_interface::msg::LanePoint>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class LanePoint
    {
    public:
        LanePoint();
        ~LanePoint();

        void point(
                const ros2_interface::msg::Point3D &_point);
        const ros2_interface::msg::Point3D &point() const;
        void theta(
            double _theta);
        double theta() const;
        void mileage(
            double _mileage);
        double mileage() const;
        void limit_speed(
            double _limit_speed);
        double limit_speed() const;
        void left_road_width(
            double _left_road_width);
        double left_road_width() const;
        void right_road_width(
            double _right_road_width);
        double right_road_width() const;
        void left_line_type(
            int _left_line_type);
        int left_line_type() const;
        void right_line_type(
            int _right_line_type);
        int right_line_type() const;
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

    class LaneInfoExtend
    {
    public:
        LaneInfoExtend();
        ~LaneInfoExtend();

        void priority(
            int _priority);
        int priority() const;
        void global_id(
            int _global_id);
        int global_id() const;
        void predecessor_ids(
            const std::vector<int> &_predecessor_ids);
        const std::vector<int>& predecessor_ids() const;
        void successor_ids(
            const std::vector<int> &_successor_ids);
        const std::vector<int>& successor_ids() const;
        void left_neighbor_id(
            int _left_neighbor_id);
        int left_neighbor_id() const;
        void right_neighbor_id(
            int _right_neighbor_id);
        int right_neighbor_id() const;
        void type(
            int _type);
        int type() const;
        void lane_points(
            const std::vector<ros2_interface::msg::LanePoint> &_lane_points);
        const std::vector<ros2_interface::msg::LanePoint>& lane_points() const;
    };



    class LanePointPubSubType
    {
    public:
        typedef LanePoint type;

        LanePointPubSubType();
        ~LanePointPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class LaneInfoExtendPubSubType
    {
    public:
        typedef LaneInfoExtend type;

        LaneInfoExtendPubSubType();
        ~LaneInfoExtendPubSubType();
    };

}
}


%template(LaneInfoExtendDataWriter) FastddsDataWriter<ros2_interface::msg::LaneInfoExtendPubSubType>;
%template(LaneInfoExtendDataReader) FastddsDataReader<ros2_interface::msg::LaneInfoExtendPubSubType>;
