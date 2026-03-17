%module fastdds_matrix_3d 
%{
#include "FastddsDataReader.hpp"
#include "FastddsDataWriter.hpp"
#include "Matrix3DPubSubTypes.h"
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
    class Matrix3D
    {
    public:
        Matrix3D();
        ~Matrix3D();

        void a00(
            double _a00);
        double a00() const;
        void a01(
            double _a01);
        double a01() const;
        void a02(
            double _a02);
        double a02() const;
        void a10(
            double _a10);
        double a10() const;
        void a11(
            double _a11);
        double a11() const;
        void a12(
            double _a12);
        double a12() const;
        void a20(
            double _a20);
        double a20() const;
        void a21(
            double _a21);
        double a21() const;
        void a22(
            double _a22);
        double a22() const;
    };



    class Matrix3DPubSubType
    {
    public:
        typedef Matrix3D type;

        Matrix3DPubSubType();
        ~Matrix3DPubSubType();
    };

}
}


%template(Matrix3DDataWriter) FastddsDataWriter<ros2_interface::msg::Matrix3DPubSubType>;
%template(Matrix3DDataReader) FastddsDataReader<ros2_interface::msg::Matrix3DPubSubType>;
