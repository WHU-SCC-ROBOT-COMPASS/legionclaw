%module fastdds_quaternion 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "QuaternionPubSubTypes.h"
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
    class Quaternion
    {
    public:
        Quaternion();
        ~Quaternion();

        void qx(
            double _qx);
        double qx() const;
        void qy(
            double _qy);
        double qy() const;
        void qz(
            double _qz);
        double qz() const;
        void qw(
            double _qw);
        double qw() const;
    };



    class QuaternionPubSubType
    {
    public:
        typedef Quaternion type;

        QuaternionPubSubType();
        ~QuaternionPubSubType();
    };

}
}


%template(QuaternionDataWriter) FastddsDataWriter<ros2_interface::msg::QuaternionPubSubType>;
%template(QuaternionDataReader) FastddsDataReader<ros2_interface::msg::QuaternionPubSubType>;
