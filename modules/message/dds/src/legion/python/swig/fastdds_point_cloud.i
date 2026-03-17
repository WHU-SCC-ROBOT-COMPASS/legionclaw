%module fastdds_point_cloud 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "PointCloudPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "PointXYZIRTPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef unsigned int uint8_t;
 typedef unsigned int uint64_t;



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
    %template(VectorPointXYZIRT) vector<ros2_interface::msg::PointXYZIRT>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class PointCloud
    {
    public:
        PointCloud();
        ~PointCloud();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void frame_id(
           const std::string &_frame_id);
        std::string frame_id() const;
        void is_dense(
            bool _is_dense);
        bool is_dense() const;
        void point(
            const std::vector<ros2_interface::msg::PointXYZIRT> &_point);
        const std::vector<ros2_interface::msg::PointXYZIRT>& point() const;
        void measurement_time(
            double _measurement_time);
        double measurement_time() const;
        void width(
            int _width);
        int width() const;
        void height(
            int _height);
        int height() const;
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

    class PointXYZIRT
    {
    public:
        PointXYZIRT();
        ~PointXYZIRT();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void z(
            double _z);
        double z() const;
        void intensity(
            int _intensity);
        int intensity() const;
        void ring_id(
            int _ring_id);
        int ring_id() const;
        void timestamp(
            int _timestamp);
        int timestamp() const;
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



    class PointCloudPubSubType
    {
    public:
        typedef PointCloud type;

        PointCloudPubSubType();
        ~PointCloudPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class PointXYZIRTPubSubType
    {
    public:
        typedef PointXYZIRT type;

        PointXYZIRTPubSubType();
        ~PointXYZIRTPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

}
}


%template(PointCloudDataWriter) FastddsDataWriter<ros2_interface::msg::PointCloudPubSubType>;
%template(PointCloudDataReader) FastddsDataReader<ros2_interface::msg::PointCloudPubSubType>;
