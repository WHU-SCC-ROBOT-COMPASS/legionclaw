%module fastdds_ultrasonic 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "UltrasonicPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "UltrasonicObstaclePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
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
    %template(VectorUltrasonicObstacle) vector<ros2_interface::msg::UltrasonicObstacle>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Ultrasonic
    {
    public:
        Ultrasonic();
        ~Ultrasonic();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void ranges(
            const std::vector<ros2_interface::msg::UltrasonicObstacle> &_ranges);
        const std::vector<ros2_interface::msg::UltrasonicObstacle>& ranges() const;
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

    class UltrasonicObstacle
    {
    public:
        UltrasonicObstacle();
        ~UltrasonicObstacle();

        void id(
            int _id);
        int id() const;
        void range(
            double _range);
        double range() const;
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



    class UltrasonicPubSubType
    {
    public:
        typedef Ultrasonic type;

        UltrasonicPubSubType();
        ~UltrasonicPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class UltrasonicObstaclePubSubType
    {
    public:
        typedef UltrasonicObstacle type;

        UltrasonicObstaclePubSubType();
        ~UltrasonicObstaclePubSubType();
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


%template(UltrasonicDataWriter) FastddsDataWriter<ros2_interface::msg::UltrasonicPubSubType>;
%template(UltrasonicDataReader) FastddsDataReader<ros2_interface::msg::UltrasonicPubSubType>;
