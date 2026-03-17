%module fastdds_obstacle_intent 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ObstacleIntentPubSubTypes.h"
%}



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
    class ObstacleIntent
    {
    public:
        ObstacleIntent();
        ~ObstacleIntent();

        void type(
            int _type);
        int type() const;
    };



    class ObstacleIntentPubSubType
    {
    public:
        typedef ObstacleIntent type;

        ObstacleIntentPubSubType();
        ~ObstacleIntentPubSubType();
    };

}
}


%template(ObstacleIntentDataWriter) FastddsDataWriter<ros2_interface::msg::ObstacleIntentPubSubType>;
%template(ObstacleIntentDataReader) FastddsDataReader<ros2_interface::msg::ObstacleIntentPubSubType>;
