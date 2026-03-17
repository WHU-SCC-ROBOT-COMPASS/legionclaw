%module fastdds_hmi_obstacle 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "HMIObstaclePubSubTypes.h"
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

    class HMIObstacle
    {
    public:
        HMIObstacle();
        ~HMIObstacle();

        void id(
            int _id);
        int id() const;
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
        void length(
            double _length);
        double length() const;
        void width(
            double _width);
        double width() const;
        void height(
            double _height);
        double height() const;
        void type(
            int _type);
        int type() const;
        void confidence(
            double _confidence);
        double confidence() const;
        void confidence_type(
            int _confidence_type);
        int confidence_type() const;
        void sub_type(
            int _sub_type);
        int sub_type() const;
        void points(
            const std::vector<ros2_interface::msg::Point3D> &_points);
        const std::vector<ros2_interface::msg::Point3D>& points() const;
        void cipv_flag(
            int _cipv_flag);
        int cipv_flag() const;
        void fusion_type(
            int _fusion_type);
        int fusion_type() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class HMIObstaclePubSubType
    {
    public:
        typedef HMIObstacle type;

        HMIObstaclePubSubType();
        ~HMIObstaclePubSubType();
    };

}
}


%template(HMIObstacleDataWriter) FastddsDataWriter<ros2_interface::msg::HMIObstaclePubSubType>;
%template(HMIObstacleDataReader) FastddsDataReader<ros2_interface::msg::HMIObstaclePubSubType>;
