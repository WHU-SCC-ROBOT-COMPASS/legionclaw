%module fastdds_parking_roi 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ParkingRoiPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "Point2dListPubSubTypes.h"
#include "Point2DPubSubTypes.h"
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
    %template(VectorPoint2dList) vector<ros2_interface::msg::Point2dList>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class ParkingRoi
    {
    public:
        ParkingRoi();
        ~ParkingRoi();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void obstacles_vec(
            const std::vector<ros2_interface::msg::Point2dList> &_obstacles_vec);
        const std::vector<ros2_interface::msg::Point2dList>& obstacles_vec() const;
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



    class ParkingRoiPubSubType
    {
    public:
        typedef ParkingRoi type;

        ParkingRoiPubSubType();
        ~ParkingRoiPubSubType();
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

}
}


%template(ParkingRoiDataWriter) FastddsDataWriter<ros2_interface::msg::ParkingRoiPubSubType>;
%template(ParkingRoiDataReader) FastddsDataReader<ros2_interface::msg::ParkingRoiPubSubType>;
