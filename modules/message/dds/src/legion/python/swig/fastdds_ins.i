%module fastdds_ins 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Point3DPubSubTypes.h"
#include "InsPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int32_t;
 typedef unsigned int uint8_t;
 typedef unsigned int uint16_t;
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

    class Ins
    {
    public:
        Ins();
        ~Ins();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
        void latitude(
            double _latitude);
        double latitude() const;
        void longitude(
            double _longitude);
        double longitude() const;
        void elevation(
            double _elevation);
        double elevation() const;
        void utm_position(
                const ros2_interface::msg::Point3D &_utm_position);
        const ros2_interface::msg::Point3D &utm_position() const;
        void utm_zone_mumber(
            int _utm_zone_mumber);
        int utm_zone_mumber() const;
        void utm_zone_char(
            int _utm_zone_char);
        int utm_zone_char() const;
        void attitude(
                const ros2_interface::msg::Point3D &_attitude);
        const ros2_interface::msg::Point3D &attitude() const;
        void linear_velocity(
                const ros2_interface::msg::Point3D &_linear_velocity);
        const ros2_interface::msg::Point3D &linear_velocity() const;
        void sd_position(
                const ros2_interface::msg::Point3D &_sd_position);
        const ros2_interface::msg::Point3D &sd_position() const;
        void sd_attitude(
                const ros2_interface::msg::Point3D &_sd_attitude);
        const ros2_interface::msg::Point3D &sd_attitude() const;
        void sd_velocity(
                const ros2_interface::msg::Point3D &_sd_velocity);
        const ros2_interface::msg::Point3D &sd_velocity() const;
        void cep68(
            double _cep68);
        double cep68() const;
        void cep95(
            double _cep95);
        double cep95() const;
        void second(
            double _second);
        double second() const;
        void sat_use_num(
            int _sat_use_num);
        int sat_use_num() const;
        void sat_in_view_num(
            int _sat_in_view_num);
        int sat_in_view_num() const;
        void solution_status(
            int _solution_status);
        int solution_status() const;
        void position_type(
            int _position_type);
        int position_type() const;
        void p_dop(
            double _p_dop);
        double p_dop() const;
        void h_dop(
            double _h_dop);
        double h_dop() const;
        void v_dop(
            double _v_dop);
        double v_dop() const;
        void attitude_dual(
                const ros2_interface::msg::Point3D &_attitude_dual);
        const ros2_interface::msg::Point3D &attitude_dual() const;
        void sd_angle_dual(
                const ros2_interface::msg::Point3D &_sd_angle_dual);
        const ros2_interface::msg::Point3D &sd_angle_dual() const;
        void base_line_length_dual(
            double _base_line_length_dual);
        double base_line_length_dual() const;
        void solution_status_dual(
            int _solution_status_dual);
        int solution_status_dual() const;
        void position_type_dual(
            int _position_type_dual);
        int position_type_dual() const;
        void solution_source_dual(
            int _solution_source_dual);
        int solution_source_dual() const;
        void aoc(
            int _aoc);
        int aoc() const;
        void rtk_baseline(
            int _rtk_baseline);
        int rtk_baseline() const;
        void angular_velocity(
                const ros2_interface::msg::Point3D &_angular_velocity);
        const ros2_interface::msg::Point3D &angular_velocity() const;
        void acceleration(
                const ros2_interface::msg::Point3D &_acceleration);
        const ros2_interface::msg::Point3D &acceleration() const;
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



    class Point3DPubSubType
    {
    public:
        typedef Point3D type;

        Point3DPubSubType();
        ~Point3DPubSubType();
    };

    class InsPubSubType
    {
    public:
        typedef Ins type;

        InsPubSubType();
        ~InsPubSubType();
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

}
}


%template(InsDataWriter) FastddsDataWriter<ros2_interface::msg::InsPubSubType>;
%template(InsDataReader) FastddsDataReader<ros2_interface::msg::InsPubSubType>;
