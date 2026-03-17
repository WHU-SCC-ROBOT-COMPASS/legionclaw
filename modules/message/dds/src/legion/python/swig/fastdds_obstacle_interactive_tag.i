%module fastdds_obstacle_interactive_tag 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ObstacleInteractiveTagPubSubTypes.h"
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
    class ObstacleInteractiveTag
    {
    public:
        ObstacleInteractiveTag();
        ~ObstacleInteractiveTag();

        void interactive_tag(
            int _interactive_tag);
        int interactive_tag() const;
    };



    class ObstacleInteractiveTagPubSubType
    {
    public:
        typedef ObstacleInteractiveTag type;

        ObstacleInteractiveTagPubSubType();
        ~ObstacleInteractiveTagPubSubType();
    };

}
}


%template(ObstacleInteractiveTagDataWriter) FastddsDataWriter<ros2_interface::msg::ObstacleInteractiveTagPubSubType>;
%template(ObstacleInteractiveTagDataReader) FastddsDataReader<ros2_interface::msg::ObstacleInteractiveTagPubSubType>;
