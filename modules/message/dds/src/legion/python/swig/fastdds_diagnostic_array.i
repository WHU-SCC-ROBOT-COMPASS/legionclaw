%module fastdds_diagnostic_array 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "DiagnosticStatusPubSubTypes.h"
#include "KeyValuesPubSubTypes.h"
#include "DiagnosticArrayPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
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
    %template(VectorDiagnosticStatus) vector<ros2_interface::msg::DiagnosticStatus>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class DiagnosticStatus
    {
    public:
        DiagnosticStatus();
        ~DiagnosticStatus();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void id(
            int _id);
        int id() const;
        void level(
            int _level);
        int level() const;
        void name(
           const std::string &_name);
        std::string name() const;
        void message(
           const std::string &_message);
        std::string message() const;
        void hardware_id(
           const std::string &_hardware_id);
        std::string hardware_id() const;
        void values(
            const std::vector<ros2_interface::msg::KeyValues> &_values);
        const std::vector<ros2_interface::msg::KeyValues>& values() const;
    };

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

    class DiagnosticArray
    {
    public:
        DiagnosticArray();
        ~DiagnosticArray();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void status(
            const std::vector<ros2_interface::msg::DiagnosticStatus> &_status);
        const std::vector<ros2_interface::msg::DiagnosticStatus>& status() const;
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



    class DiagnosticStatusPubSubType
    {
    public:
        typedef DiagnosticStatus type;

        DiagnosticStatusPubSubType();
        ~DiagnosticStatusPubSubType();
    };

    class KeyValuesPubSubType
    {
    public:
        typedef KeyValues type;

        KeyValuesPubSubType();
        ~KeyValuesPubSubType();
    };

    class DiagnosticArrayPubSubType
    {
    public:
        typedef DiagnosticArray type;

        DiagnosticArrayPubSubType();
        ~DiagnosticArrayPubSubType();
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


%template(DiagnosticArrayDataWriter) FastddsDataWriter<ros2_interface::msg::DiagnosticArrayPubSubType>;
%template(DiagnosticArrayDataReader) FastddsDataReader<ros2_interface::msg::DiagnosticArrayPubSubType>;
