%module fastdds_stories 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "StoriesPubSubTypes.h"
%}
%include "std_string.i"



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
    class Stories
    {
    public:
        Stories();
        ~Stories();

        void note(
           const std::string &_note);
        std::string note() const;
    };



    class StoriesPubSubType
    {
    public:
        typedef Stories type;

        StoriesPubSubType();
        ~StoriesPubSubType();
    };

}
}


%template(StoriesDataWriter) FastddsDataWriter<ros2_interface::msg::StoriesPubSubType>;
%template(StoriesDataReader) FastddsDataReader<ros2_interface::msg::StoriesPubSubType>;
