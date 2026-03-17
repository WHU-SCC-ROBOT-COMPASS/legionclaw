%module fastdds_vehicle_signal 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "VehicleSignalPubSubTypes.h"
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
    class VehicleSignal
    {
    public:
        VehicleSignal();
        ~VehicleSignal();

        void turn_signal(
            int _turn_signal);
        int turn_signal() const;
        void high_beam(
            bool _high_beam);
        bool high_beam() const;
        void low_beam(
            bool _low_beam);
        bool low_beam() const;
        void horn(
            bool _horn);
        bool horn() const;
        void emergency_light(
            bool _emergency_light);
        bool emergency_light() const;
    };



    class VehicleSignalPubSubType
    {
    public:
        typedef VehicleSignal type;

        VehicleSignalPubSubType();
        ~VehicleSignalPubSubType();
    };

}
}


%template(VehicleSignalDataWriter) FastddsDataWriter<ros2_interface::msg::VehicleSignalPubSubType>;
%template(VehicleSignalDataReader) FastddsDataReader<ros2_interface::msg::VehicleSignalPubSubType>;
