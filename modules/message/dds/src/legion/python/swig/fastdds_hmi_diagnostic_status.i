%module fastdds_hmi_diagnostic_status 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HMIDiagnosticStatusPubSubTypes.h"
%}
%include "std_string.i"
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
    class HMIDiagnosticStatus
    {
    public:
        HMIDiagnosticStatus();
        ~HMIDiagnosticStatus();

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
    };



    class HMIDiagnosticStatusPubSubType
    {
    public:
        typedef HMIDiagnosticStatus type;

        HMIDiagnosticStatusPubSubType();
        ~HMIDiagnosticStatusPubSubType();
    };

}
}


%template(HMIDiagnosticStatusDataWriter) FastddsDataWriter<ros2_interface::msg::HMIDiagnosticStatusPubSubType>;
%template(HMIDiagnosticStatusDataReader) FastddsDataReader<ros2_interface::msg::HMIDiagnosticStatusPubSubType>;
