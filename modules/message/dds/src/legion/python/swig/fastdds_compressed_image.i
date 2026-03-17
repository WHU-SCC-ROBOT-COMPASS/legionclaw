%module fastdds_compressed_image 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "CompressedImagePubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;
 typedef int int8_t;



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

    class CompressedImage
    {
    public:
        CompressedImage();
        ~CompressedImage();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void frame_id(
           const std::string &_frame_id);
        std::string frame_id() const;
        void format(
           const std::string &_format);
        std::string format() const;
        void data(
            const std::vector<int> &_data);
        const std::vector<int>& data() const;
        void measurement_time(
            double _measurement_time);
        double measurement_time() const;
        void frame_type(
            int _frame_type);
        int frame_type() const;
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

    class CompressedImagePubSubType
    {
    public:
        typedef CompressedImage type;

        CompressedImagePubSubType();
        ~CompressedImagePubSubType();
    };

}
}


%template(CompressedImageDataWriter) FastddsDataWriter<ros2_interface::msg::CompressedImagePubSubType>;
%template(CompressedImageDataReader) FastddsDataReader<ros2_interface::msg::CompressedImagePubSubType>;
