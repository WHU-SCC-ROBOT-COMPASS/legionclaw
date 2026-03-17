%module fastdds_uss_parking_info_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "UssParkingInfoListPubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "UssParkingInfoPubSubTypes.h"
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
    %template(VectorUssParkingInfo) vector<ros2_interface::msg::UssParkingInfo>;
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

    class UssParkingInfoList
    {
    public:
        UssParkingInfoList();
        ~UssParkingInfoList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void uss_parking_info(
            const std::vector<ros2_interface::msg::UssParkingInfo> &_uss_parking_info);
        const std::vector<ros2_interface::msg::UssParkingInfo>& uss_parking_info() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
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

    class UssParkingInfo
    {
    public:
        UssParkingInfo();
        ~UssParkingInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void uss_parking_type(
            int _uss_parking_type);
        int uss_parking_type() const;
        void parking_point0(
                const ros2_interface::msg::Point3D &_parking_point0);
        const ros2_interface::msg::Point3D &parking_point0() const;
        void parking_point1(
                const ros2_interface::msg::Point3D &_parking_point1);
        const ros2_interface::msg::Point3D &parking_point1() const;
        void parking_point2(
                const ros2_interface::msg::Point3D &_parking_point2);
        const ros2_interface::msg::Point3D &parking_point2() const;
        void parking_point3(
                const ros2_interface::msg::Point3D &_parking_point3);
        const ros2_interface::msg::Point3D &parking_point3() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class UssParkingInfoListPubSubType
    {
    public:
        typedef UssParkingInfoList type;

        UssParkingInfoListPubSubType();
        ~UssParkingInfoListPubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class UssParkingInfoPubSubType
    {
    public:
        typedef UssParkingInfo type;

        UssParkingInfoPubSubType();
        ~UssParkingInfoPubSubType();
    };

}
}


%template(UssParkingInfoListDataWriter) FastddsDataWriter<ros2_interface::msg::UssParkingInfoListPubSubType>;
%template(UssParkingInfoListDataReader) FastddsDataReader<ros2_interface::msg::UssParkingInfoListPubSubType>;
