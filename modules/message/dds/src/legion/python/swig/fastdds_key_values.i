%module fastdds_key_values 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "KeyValuesPubSubTypes.h"
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
    class KeyValues
    {
    public:
        KeyValues();
        ~KeyValues();

        void key(
           const std::string &_key);
        std::string key() const;
        void value(
           const std::string &_value);
        std::string value() const;
    };



    class KeyValuesPubSubType
    {
    public:
        typedef KeyValues type;

        KeyValuesPubSubType();
        ~KeyValuesPubSubType();
    };

}
}


%template(KeyValuesDataWriter) FastddsDataWriter<ros2_interface::msg::KeyValuesPubSubType>;
%template(KeyValuesDataReader) FastddsDataReader<ros2_interface::msg::KeyValuesPubSubType>;
