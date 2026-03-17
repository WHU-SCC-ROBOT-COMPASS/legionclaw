%module fastdds_faults 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "FaultsPubSubTypes.h"
#include "TimePubSubTypes.h"
#include "FaultPubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef int int32_t;
 typedef unsigned int uint32_t;
 typedef unsigned int uint64_t;



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
    %template(VectorFault) vector<ros2_interface::msg::Fault>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Faults
    {
    public:
        Faults();
        ~Faults();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void version(
            int _version);
        int version() const;
        void app_id(
            int _app_id);
        int app_id() const;
        void is_active(
            bool _is_active);
        bool is_active() const;
        void faults(
            const std::vector<ros2_interface::msg::Fault> &_faults);
        const std::vector<ros2_interface::msg::Fault>& faults() const;
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

    class Fault
    {
    public:
        Fault();
        ~Fault();

        void timestamp(
                const ros2_interface::msg::Time &_timestamp);
        const ros2_interface::msg::Time &timestamp() const;
        void code(
            int _code);
        int code() const;
        void reason(
           const std::string &_reason);
        std::string reason() const;
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



    class FaultsPubSubType
    {
    public:
        typedef Faults type;

        FaultsPubSubType();
        ~FaultsPubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
    };

    class FaultPubSubType
    {
    public:
        typedef Fault type;

        FaultPubSubType();
        ~FaultPubSubType();
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


%template(FaultsDataWriter) FastddsDataWriter<ros2_interface::msg::FaultsPubSubType>;
%template(FaultsDataReader) FastddsDataReader<ros2_interface::msg::FaultsPubSubType>;
