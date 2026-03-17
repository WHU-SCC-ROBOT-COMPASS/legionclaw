%module fastdds_junction_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "JunctionInfoPubSubTypes.h"
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

    class JunctionInfo
    {
    public:
        JunctionInfo();
        ~JunctionInfo();

        void id(
            int _id);
        int id() const;
        void light_flag(
            int _light_flag);
        int light_flag() const;
        void light_color(
            int _light_color);
        int light_color() const;
        void light_remain_time(
            double _light_remain_time);
        double light_remain_time() const;
        void distance_to_stop(
            double _distance_to_stop);
        double distance_to_stop() const;
        void direction_flag(
            int _direction_flag);
        int direction_flag() const;
        void direction(
            int _direction);
        int direction() const;
        void distance_to_junction(
            double _distance_to_junction);
        double distance_to_junction() const;
        void stop_line(
            const std::vector<ros2_interface::msg::Point3D> &_stop_line);
        const std::vector<ros2_interface::msg::Point3D>& stop_line() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class JunctionInfoPubSubType
    {
    public:
        typedef JunctionInfo type;

        JunctionInfoPubSubType();
        ~JunctionInfoPubSubType();
    };

}
}


%template(JunctionInfoDataWriter) FastddsDataWriter<ros2_interface::msg::JunctionInfoPubSubType>;
%template(JunctionInfoDataReader) FastddsDataReader<ros2_interface::msg::JunctionInfoPubSubType>;
