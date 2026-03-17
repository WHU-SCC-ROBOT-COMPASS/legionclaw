%module fastdds_image_key_point 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImageKeyPointPubSubTypes.h"
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
    class ImageKeyPoint
    {
    public:
        ImageKeyPoint();
        ~ImageKeyPoint();

        void x(
            double _x);
        double x() const;
        void y(
            double _y);
        double y() const;
        void confidence(
            double _confidence);
        double confidence() const;
    };



    class ImageKeyPointPubSubType
    {
    public:
        typedef ImageKeyPoint type;

        ImageKeyPointPubSubType();
        ~ImageKeyPointPubSubType();
    };

}
}


%template(ImageKeyPointDataWriter) FastddsDataWriter<ros2_interface::msg::ImageKeyPointPubSubType>;
%template(ImageKeyPointDataReader) FastddsDataReader<ros2_interface::msg::ImageKeyPointPubSubType>;
