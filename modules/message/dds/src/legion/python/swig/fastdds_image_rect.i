%module fastdds_image_rect 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImageRectPubSubTypes.h"
%}
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
    class ImageRect
    {
    public:
        ImageRect();
        ~ImageRect();

        void x(
            int _x);
        int x() const;
        void y(
            int _y);
        int y() const;
        void width(
            int _width);
        int width() const;
        void height(
            int _height);
        int height() const;
    };



    class ImageRectPubSubType
    {
    public:
        typedef ImageRect type;

        ImageRectPubSubType();
        ~ImageRectPubSubType();
    };

}
}


%template(ImageRectDataWriter) FastddsDataWriter<ros2_interface::msg::ImageRectPubSubType>;
%template(ImageRectDataReader) FastddsDataReader<ros2_interface::msg::ImageRectPubSubType>;
