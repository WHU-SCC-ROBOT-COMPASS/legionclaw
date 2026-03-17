%module fastdds_uss_obstacle_list 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "UssObstaclePubSubTypes.h"
#include "TimePubSubTypes.h"
#include "HeaderPubSubTypes.h"
#include "UssObstacleListPubSubTypes.h"
%}
%include "std_string.i"
%include "std_vector.i"
 typedef unsigned int uint32_t;



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
    %template(VectorUssObstacle) vector<ros2_interface::msg::UssObstacle>;
    %template(Vectordouble) vector<double>;
}
%module ros2_interface
%module msg
namespace ros2_interface{
namespace msg{
    class UssObstacle
    {
    public:
        UssObstacle();
        ~UssObstacle();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void uss_obstacle_region(
            int _uss_obstacle_region);
        int uss_obstacle_region() const;
        void mx(
            double _mx);
        double mx() const;
        void my(
            double _my);
        double my() const;
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

    class UssObstacleList
    {
    public:
        UssObstacleList();
        ~UssObstacleList();

        void header(
                const ros2_interface::msg::Header &_header);
        const ros2_interface::msg::Header &header() const;
        void uss_obstacles(
            const std::vector<ros2_interface::msg::UssObstacle> &_uss_obstacles);
        const std::vector<ros2_interface::msg::UssObstacle>& uss_obstacles() const;
        void uss_ranges(
            const std::vector<double> &_uss_ranges);
        const std::vector<double>& uss_ranges() const;
        void error_code(
            int _error_code);
        int error_code() const;
        void is_valid(
            bool _is_valid);
        bool is_valid() const;
    };



    class UssObstaclePubSubType
    {
    public:
        typedef UssObstacle type;

        UssObstaclePubSubType();
        ~UssObstaclePubSubType();
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

    class UssObstacleListPubSubType
    {
    public:
        typedef UssObstacleList type;

        UssObstacleListPubSubType();
        ~UssObstacleListPubSubType();
    };

}
}


%template(UssObstacleListDataWriter) FastddsDataWriter<ros2_interface::msg::UssObstacleListPubSubType>;
%template(UssObstacleListDataReader) FastddsDataReader<ros2_interface::msg::UssObstacleListPubSubType>;
