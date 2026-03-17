%module fastdds_sl_boundary 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "SLBoundaryPubSubTypes.h"
#include "SLPointPubSubTypes.h"
%}
%include "std_vector.i"



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
    %template(VectorSLPoint) vector<ros2_interface::msg::SLPoint>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class SLBoundary
    {
    public:
        SLBoundary();
        ~SLBoundary();

        void start_s(
            double _start_s);
        double start_s() const;
        void end_s(
            double _end_s);
        double end_s() const;
        void start_l(
            double _start_l);
        double start_l() const;
        void end_l(
            double _end_l);
        double end_l() const;
        void boundary_point(
            const std::vector<ros2_interface::msg::SLPoint> &_boundary_point);
        const std::vector<ros2_interface::msg::SLPoint>& boundary_point() const;
    };

    class SLPoint
    {
    public:
        SLPoint();
        ~SLPoint();

        void s(
            double _s);
        double s() const;
        void l(
            double _l);
        double l() const;
    };



    class SLBoundaryPubSubType
    {
    public:
        typedef SLBoundary type;

        SLBoundaryPubSubType();
        ~SLBoundaryPubSubType();
    };

    class SLPointPubSubType
    {
    public:
        typedef SLPoint type;

        SLPointPubSubType();
        ~SLPointPubSubType();
    };

}
}


%template(SLBoundaryDataWriter) FastddsDataWriter<ros2_interface::msg::SLBoundaryPubSubType>;
%template(SLBoundaryDataReader) FastddsDataReader<ros2_interface::msg::SLBoundaryPubSubType>;
