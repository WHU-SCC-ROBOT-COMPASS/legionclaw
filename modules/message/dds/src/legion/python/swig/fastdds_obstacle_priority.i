%module fastdds_obstacle_priority 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ObstaclePriorityPubSubTypes.h"
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
    class ObstaclePriority
    {
    public:
        ObstaclePriority();
        ~ObstaclePriority();

        void priority(
            int _priority);
        int priority() const;
    };



    class ObstaclePriorityPubSubType
    {
    public:
        typedef ObstaclePriority type;

        ObstaclePriorityPubSubType();
        ~ObstaclePriorityPubSubType();
    };

}
}


%template(ObstaclePriorityDataWriter) FastddsDataWriter<ros2_interface::msg::ObstaclePriorityPubSubType>;
%template(ObstaclePriorityDataReader) FastddsDataReader<ros2_interface::msg::ObstaclePriorityPubSubType>;
