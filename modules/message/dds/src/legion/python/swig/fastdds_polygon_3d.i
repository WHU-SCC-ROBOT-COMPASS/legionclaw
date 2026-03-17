%module fastdds_polygon_3d 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Polygon3DPubSubTypes.h"
#include "Point3DPubSubTypes.h"
%}
%include "std_vector.i"



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
    class Polygon3D
    {
    public:
        Polygon3D();
        ~Polygon3D();

        void coordinate_system(
            int _coordinate_system);
        int coordinate_system() const;
        void points(
            const std::vector<ros2_interface::msg::Point3D> &_points);
        const std::vector<ros2_interface::msg::Point3D>& points() const;
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



    class Polygon3DPubSubType
    {
    public:
        typedef Polygon3D type;

        Polygon3DPubSubType();
        ~Polygon3DPubSubType();
    };

    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

}
}


%template(Polygon3DDataWriter) FastddsDataWriter<ros2_interface::msg::Polygon3DPubSubType>;
%template(Polygon3DDataReader) FastddsDataReader<ros2_interface::msg::Polygon3DPubSubType>;
