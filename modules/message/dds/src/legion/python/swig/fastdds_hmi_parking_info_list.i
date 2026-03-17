%module fastdds_hmi_parking_info_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "HMIParkingInfoListPubSubTypes.h"
#include "Polygon3DPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "HMIParkingInfoPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
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
    %template(VectorHMIParkingInfo) vector<ros2_interface::msg::HMIParkingInfo>;
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

    class HMIParkingInfoList
    {
    public:
        HMIParkingInfoList();
        ~HMIParkingInfoList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void parking_info(
            const std::vector<ros2_interface::msg::HMIParkingInfo> &_parking_info);
        const std::vector<ros2_interface::msg::HMIParkingInfo>& parking_info() const;
        void parking_out_enable(
            bool _parking_out_enable);
        bool parking_out_enable() const;
        void parking_out_direction(
            int _parking_out_direction);
        int parking_out_direction() const;
        void parking_out_id(
            int _parking_out_id);
        int parking_out_id() const;
    };

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

    class HMIParkingInfo
    {
    public:
        HMIParkingInfo();
        ~HMIParkingInfo();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void parking_space_id(
            int _parking_space_id);
        int parking_space_id() const;
        void parking_type(
            int _parking_type);
        int parking_type() const;
        void parking_status(
            int _parking_status);
        int parking_status() const;
        void center_point_of_parking(
                const ros2_interface::msg::Point3D &_center_point_of_parking);
        const ros2_interface::msg::Point3D &center_point_of_parking() const;
        void theta(
            double _theta);
        double theta() const;
        void width(
            double _width);
        double width() const;
        void length(
            double _length);
        double length() const;
        void is_custom_parking(
            bool _is_custom_parking);
        bool is_custom_parking() const;
        void polygon(
                const ros2_interface::msg::Polygon3D &_polygon);
        const ros2_interface::msg::Polygon3D &polygon() const;
        void parking_source_type(
            int _parking_source_type);
        int parking_source_type() const;
    };



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class HMIParkingInfoListPubSubType
    {
    public:
        typedef HMIParkingInfoList type;

        HMIParkingInfoListPubSubType();
        ~HMIParkingInfoListPubSubType();
    };

    class Polygon3DPubSubType
    {
    public:
        typedef Polygon3D type;

        Polygon3DPubSubType();
        ~Polygon3DPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class HeaderPubSubType
    {
    public:
        typedef Header type;

        HeaderPubSubType();
        ~HeaderPubSubType();
    };

    class HMIParkingInfoPubSubType
    {
    public:
        typedef HMIParkingInfo type;

        HMIParkingInfoPubSubType();
        ~HMIParkingInfoPubSubType();
    };

}
}


%template(HMIParkingInfoListDataWriter) FastddsDataWriter<ros2_interface::msg::HMIParkingInfoListPubSubType>;
%template(HMIParkingInfoListDataReader) FastddsDataReader<ros2_interface::msg::HMIParkingInfoListPubSubType>;
