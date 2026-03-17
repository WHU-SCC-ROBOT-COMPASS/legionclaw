%module fastdds_hmi_diagnostic_array 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "HMIDiagnosticStatusPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HMIDiagnosticArrayPubSubTypes.h"
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
    %template(VectorHMIDiagnosticStatus) vector<ros2_interface::msg::HMIDiagnosticStatus>;
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

    class HMIDiagnosticArray
    {
    public:
        HMIDiagnosticArray();
        ~HMIDiagnosticArray();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void status(
            const std::vector<ros2_interface::msg::HMIDiagnosticStatus> &_status);
        const std::vector<ros2_interface::msg::HMIDiagnosticStatus>& status() const;
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



    class HMIDiagnosticStatusPubSubType
    {
    public:
        typedef HMIDiagnosticStatus type;

        HMIDiagnosticStatusPubSubType();
        ~HMIDiagnosticStatusPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class HMIDiagnosticArrayPubSubType
    {
    public:
        typedef HMIDiagnosticArray type;

        HMIDiagnosticArrayPubSubType();
        ~HMIDiagnosticArrayPubSubType();
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


%template(HMIDiagnosticArrayDataWriter) FastddsDataWriter<ros2_interface::msg::HMIDiagnosticArrayPubSubType>;
%template(HMIDiagnosticArrayDataReader) FastddsDataReader<ros2_interface::msg::HMIDiagnosticArrayPubSubType>;
