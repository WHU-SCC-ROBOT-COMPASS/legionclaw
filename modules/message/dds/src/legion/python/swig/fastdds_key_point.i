%module fastdds_key_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "KeyPointPubSubTypes.h"
%}
%include "std_string.i"
 typedef int int16_t;



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
    class KeyPoint
    {
    public:
        KeyPoint();
        ~KeyPoint();

        void id(
            int _id);
        int id() const;
        void latitude(
            double _latitude);
        double latitude() const;
        void longitude(
            double _longitude);
        double longitude() const;
        void ele(
            double _ele);
        double ele() const;
        void heading(
            double _heading);
        double heading() const;
        void name(
           const std::string &_name);
        std::string name() const;
    };



    class KeyPointPubSubType
    {
    public:
        typedef KeyPoint type;

        KeyPointPubSubType();
        ~KeyPointPubSubType();
    };

}
}


%template(KeyPointDataWriter) FastddsDataWriter<ros2_interface::msg::KeyPointPubSubType>;
%template(KeyPointDataReader) FastddsDataReader<ros2_interface::msg::KeyPointPubSubType>;
