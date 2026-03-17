%module fastdds_bbox_2d 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "BBox2DPubSubTypes.h"
%}
 typedef int int16_t;



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
    class BBox2D
    {
    public:
        BBox2D();
        ~BBox2D();

        void xmin(
            int _xmin);
        int xmin() const;
        void ymin(
            int _ymin);
        int ymin() const;
        void xmax(
            int _xmax);
        int xmax() const;
        void ymax(
            int _ymax);
        int ymax() const;
    };



    class BBox2DPubSubType
    {
    public:
        typedef BBox2D type;

        BBox2DPubSubType();
        ~BBox2DPubSubType();
    };

}
}


%template(BBox2DDataWriter) FastddsDataWriter<ros2_interface::msg::BBox2DPubSubType>;
%template(BBox2DDataReader) FastddsDataReader<ros2_interface::msg::BBox2DPubSubType>;
