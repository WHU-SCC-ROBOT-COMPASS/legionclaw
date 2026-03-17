%module fastdds_ultrasonic_obstacle 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "UltrasonicObstaclePubSubTypes.h"
%}
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
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
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



    class UltrasonicObstaclePubSubType
    {
    public:
        typedef UltrasonicObstacle type;

        UltrasonicObstaclePubSubType();
        ~UltrasonicObstaclePubSubType();
    };

}
}


%template(UltrasonicObstacleDataWriter) FastddsDataWriter<ros2_interface::msg::UltrasonicObstaclePubSubType>;
%template(UltrasonicObstacleDataReader) FastddsDataReader<ros2_interface::msg::UltrasonicObstaclePubSubType>;
