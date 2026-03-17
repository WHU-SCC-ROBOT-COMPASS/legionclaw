%module fastdds_gaussian_info 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "GaussianInfoPubSubTypes.h"
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
    class GaussianInfo
    {
    public:
        GaussianInfo();
        ~GaussianInfo();

        void sigma_x(
            double _sigma_x);
        double sigma_x() const;
        void sigma_y(
            double _sigma_y);
        double sigma_y() const;
        void correlation(
            double _correlation);
        double correlation() const;
        void area_probability(
            double _area_probability);
        double area_probability() const;
        void ellipse_a(
            double _ellipse_a);
        double ellipse_a() const;
        void ellipse_b(
            double _ellipse_b);
        double ellipse_b() const;
        void theta_a(
            double _theta_a);
        double theta_a() const;
    };



    class GaussianInfoPubSubType
    {
    public:
        typedef GaussianInfo type;

        GaussianInfoPubSubType();
        ~GaussianInfoPubSubType();
    };

}
}


%template(GaussianInfoDataWriter) FastddsDataWriter<ros2_interface::msg::GaussianInfoPubSubType>;
%template(GaussianInfoDataReader) FastddsDataReader<ros2_interface::msg::GaussianInfoPubSubType>;
