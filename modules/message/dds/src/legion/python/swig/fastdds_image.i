%module fastdds_image 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "ImagePubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef unsigned int uint8_t;



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
    %template(Vectorint) vector<int>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class Image
    {
    public:
        Image();
        ~Image();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void height(
            int _height);
        int height() const;
        void width(
            int _width);
        int width() const;
        void encoding(
           const std::string &_encoding);
        std::string encoding() const;
        void is_bigendian(
            int _is_bigendian);
        int is_bigendian() const;
        void step(
            int _step);
        int step() const;
        void data(
            const std::vector<int> &_data);
        const std::vector<int>& data() const;
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



    class ImagePubSubType
    {
    public:
        typedef Image type;

        ImagePubSubType();
        ~ImagePubSubType();
    };

    class TimePubSubType
    {
    public:
        typedef Time type;

        TimePubSubType();
        ~TimePubSubType();
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


%template(ImageDataWriter) FastddsDataWriter<ros2_interface::msg::ImagePubSubType>;
%template(ImageDataReader) FastddsDataReader<ros2_interface::msg::ImagePubSubType>;
