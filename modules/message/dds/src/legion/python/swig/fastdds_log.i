%module fastdds_log 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "LogPubSubTypes.h"
#include "TimePubSubTypes.h"
%}
%include "std_string.i"
 typedef unsigned int uint8_t;
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
    class Log
    {
    public:
        Log();
        ~Log();

        void stamp(
                const ros2_interface::msg::Time &_stamp);
        const ros2_interface::msg::Time &stamp() const;
        void level(
            int _level);
        int level() const;
        void name(
           const std::string &_name);
        std::string name() const;
        void msg(
           const std::string &_msg);
        std::string msg() const;
        void file(
           const std::string &_file);
        std::string file() const;
        void function(
           const std::string &_function);
        std::string function() const;
        void line(
            int _line);
        int line() const;
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



    class LogPubSubType
    {
    public:
        typedef Log type;

        LogPubSubType();
        ~LogPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

}
}


%template(LogDataWriter) FastddsDataWriter<ros2_interface::msg::LogPubSubType>;
%template(LogDataReader) FastddsDataReader<ros2_interface::msg::LogPubSubType>;
