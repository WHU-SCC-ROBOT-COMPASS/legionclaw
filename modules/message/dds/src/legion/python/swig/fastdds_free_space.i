%module fastdds_free_space 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Polygon2DPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "Point2DPubSubTypes.h"
#include "FreeSpacePubSubTypes.h"
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
    %template(VectorPolygon2D) vector<ros2_interface::msg::Polygon2D>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
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

    class FreeSpace
    {
    public:
        FreeSpace();
        ~FreeSpace();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void sensor_id(
            int _sensor_id);
        int sensor_id() const;
        void freespace_region(
            const std::vector<ros2_interface::msg::Polygon2D> &_freespace_region);
        const std::vector<ros2_interface::msg::Polygon2D>& freespace_region() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
    };



    class Polygon2DPubSubType
    {
    public:
        typedef Polygon2D type;

        Polygon2DPubSubType();
        ~Polygon2DPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class Point2DPubSubType
    {
    public:
        typedef Point2D type;

        Point2DPubSubType();
        ~Point2DPubSubType();
    };

    class FreeSpacePubSubType
    {
    public:
        typedef FreeSpace type;

        FreeSpacePubSubType();
        ~FreeSpacePubSubType();
    };

}
}


%template(FreeSpaceDataWriter) FastddsDataWriter<ros2_interface::msg::FreeSpacePubSubType>;
%template(FreeSpaceDataReader) FastddsDataReader<ros2_interface::msg::FreeSpacePubSubType>;
