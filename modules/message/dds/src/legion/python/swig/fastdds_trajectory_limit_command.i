%module fastdds_trajectory_limit_command 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TrajectoryLimitCommandPubSubTypes.h"
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
    class TrajectoryLimitCommand
    {
    public:
        TrajectoryLimitCommand();
        ~TrajectoryLimitCommand();

        void speed_limit_enable(
            bool _speed_limit_enable);
        bool speed_limit_enable() const;
        void speed_limit(
            double _speed_limit);
        double speed_limit() const;
        void kappa_limit_enable(
            bool _kappa_limit_enable);
        bool kappa_limit_enable() const;
        void kappa_limit(
            double _kappa_limit);
        double kappa_limit() const;
    };



    class TrajectoryLimitCommandPubSubType
    {
    public:
        typedef TrajectoryLimitCommand type;

        TrajectoryLimitCommandPubSubType();
        ~TrajectoryLimitCommandPubSubType();
    };

}
}


%template(TrajectoryLimitCommandDataWriter) FastddsDataWriter<ros2_interface::msg::TrajectoryLimitCommandPubSubType>;
%template(TrajectoryLimitCommandDataReader) FastddsDataReader<ros2_interface::msg::TrajectoryLimitCommandPubSubType>;
