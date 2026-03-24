#include "multi_lidar_splicing.h"
#include <stdexcept>
#include <algorithm>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
constexpr bool system_is_bigendian = true;
#else
constexpr bool system_is_bigendian = false;
#endif

template <typename T>
void swapBytes(T& value);

template <>
void swapBytes<int16_t>(int16_t& value) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    std::swap(bytes[0], bytes[1]);
}

template <>
void swapBytes<uint16_t>(uint16_t& value) { swapBytes(reinterpret_cast<int16_t&>(value)); }

template <>
void swapBytes<int32_t>(int32_t& value) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    std::swap(bytes[0], bytes[3]);
    std::swap(bytes[1], bytes[2]);
}

template <>
void swapBytes<uint32_t>(uint32_t& value) { swapBytes(reinterpret_cast<int32_t&>(value)); }

template <>
void swapBytes<float>(float& value) { swapBytes(reinterpret_cast<uint32_t&>(value)); }

template <>
void swapBytes<double>(double& value) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    std::swap(bytes[0], bytes[7]);
    std::swap(bytes[1], bytes[6]);
    std::swap(bytes[2], bytes[5]);
    std::swap(bytes[3], bytes[4]);
}

template <typename T>
T readValue(const uint8_t* data_ptr, int offset, int datatype, bool is_bigendian) {
    T value = 0;
    using PF = sensor_msgs::msg::PointField;

    switch (datatype) {
        case PF::INT8:
            value = static_cast<T>(*reinterpret_cast<const int8_t*>(data_ptr + offset));
            break;
        case PF::UINT8:
            value = static_cast<T>(*reinterpret_cast<const uint8_t*>(data_ptr + offset));
            break;
        case PF::INT16: {
            int16_t val;
            memcpy(&val, data_ptr + offset, sizeof(int16_t));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        case PF::UINT16: {
            uint16_t val;
            memcpy(&val, data_ptr + offset, sizeof(uint16_t));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        case PF::INT32: {
            int32_t val;
            memcpy(&val, data_ptr + offset, sizeof(int32_t));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        case PF::UINT32: {
            uint32_t val;
            memcpy(&val, data_ptr + offset, sizeof(uint32_t));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        case PF::FLOAT32: {
            float val;
            memcpy(&val, data_ptr + offset, sizeof(float));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        case PF::FLOAT64: {
            double val;
            memcpy(&val, data_ptr + offset, sizeof(double));
            if (is_bigendian != system_is_bigendian) swapBytes(val);
            value = static_cast<T>(val);
            break;
        }
        default:
            throw std::runtime_error("Unsupported field datatype");
    }
    return value;
}

// 与ROS1一致：自定义 fromROSMsg，确保 intensity 正确
static void fromROSMsg(const sensor_msgs::msg::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    // 基本信息
    cloud.header.stamp    = rclcpp::Time(cloud_msg.header.stamp).nanoseconds() / 1000; // microseconds
    cloud.header.frame_id = cloud_msg.header.frame_id;
    cloud.width  = cloud_msg.width;
    cloud.height = cloud_msg.height;
    cloud.is_dense = cloud_msg.is_dense != 0;
    cloud.points.resize(cloud_msg.width * cloud_msg.height);

    struct FieldInfo { int offset{-1}; int datatype{0}; };
    FieldInfo x_field, y_field, z_field, intensity_field;

    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x" || field.name == "X") x_field = {static_cast<int>(field.offset), field.datatype};
        else if (field.name == "y" || field.name == "Y") y_field = {static_cast<int>(field.offset), field.datatype};
        else if (field.name == "z" || field.name == "Z") z_field = {static_cast<int>(field.offset), field.datatype};
        else if (field.name == "intensity" || field.name == "i" || field.name == "I") intensity_field = {static_cast<int>(field.offset), field.datatype};
    }
    if (x_field.offset == -1 || y_field.offset == -1 || z_field.offset == -1) {
        throw std::runtime_error("Missing required x/y/z fields");
    }

    const uint8_t* ros_data = cloud_msg.data.data();
    const int point_step = static_cast<int>(cloud_msg.point_step);

    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint8_t* ros_point = ros_data + i * point_step;
        auto& point = cloud[i];

        point.x = readValue<float>(ros_point, x_field.offset, x_field.datatype, cloud_msg.is_bigendian);
        point.y = readValue<float>(ros_point, y_field.offset, y_field.datatype, cloud_msg.is_bigendian);
        point.z = readValue<float>(ros_point, z_field.offset, z_field.datatype, cloud_msg.is_bigendian);
        if (intensity_field.offset != -1)
            point.intensity = readValue<float>(ros_point, intensity_field.offset, intensity_field.datatype, cloud_msg.is_bigendian);
        else
            point.intensity = 0.0f;
    }
}


MultiLidarSplicing::MultiLidarSplicing(std::string lidar_front,
                                       std::string lidar_mid,
                                       std::string lidar_left,
                                       std::string lidar_right,
                                       std::string lidar_back,
                                       std::string frame_id,
                                       std::string publish_topic)
: rclcpp::Node("MultiLidarSplicing"),
  lidar_front_(std::move(lidar_front)),
  lidar_mid_(std::move(lidar_mid)),
  lidar_left_(std::move(lidar_left)),
  lidar_right_(std::move(lidar_right)),
  lidar_back_(std::move(lidar_back)),
  frame_id_(std::move(frame_id)),
  publish_topic_(std::move(publish_topic))
{
    // 读取use_sim_time参数（不声明，因为它是全局参数，可能已通过命令行设置）
    // 如果通过命令行设置了use_sim_time，则使用命令行值；否则默认为false
    bool use_sim_time = false;
    try {
        // 尝试获取use_sim_time参数（可能已通过命令行设置）
        if (this->has_parameter("use_sim_time")) {
            use_sim_time = this->get_parameter("use_sim_time").as_bool();
        }
    } catch (const std::exception& e) {
        // 如果获取失败，使用默认值false
        use_sim_time = false;
    }
    
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "使用仿真时间 (use_sim_time=true)");
        // 在ROS2中，设置use_sim_time参数后，节点会自动使用/clock话题的时间
        // 这需要ros2 run时传递参数：--ros-args -p use_sim_time:=true
    } else {
        RCLCPP_INFO(this->get_logger(), "使用系统时间 (use_sim_time=false)");
    }
    
    // 使用与订阅者兼容的QoS策略（与sensor_data profile兼容）
    // 基于sensor_data profile，但增加队列大小到100，避免消息堆积导致卡顿
    // 必须使用reliable策略，否则与使用sensor_data QoS的订阅者不兼容
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(100));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  // 与sensor_data兼容
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, qos);
}

MultiLidarSplicing::~MultiLidarSplicing() = default;

void MultiLidarSplicing::run()
{
    // 使用可靠QoS避免单点云丢包导致整帧同步失败
    rmw_qos_profile_t reliable_sensor_qos = rmw_qos_profile_sensor_data;
    reliable_sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    reliable_sensor_qos.depth = 100;

    sub_front_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_front_.getChannel(), reliable_sensor_qos);
    sub_mid_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_mid_.getChannel(),  reliable_sensor_qos);
    sub_left_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_left_.getChannel(), reliable_sensor_qos);
    sub_right_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_right_.getChannel(),  reliable_sensor_qos);
    sub_back_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_back_.getChannel(),  reliable_sensor_qos);

    // 优化同步策略：
    // 1. 队列大小设置为50，兼顾缓冲与延迟（可根据场景再调大）
    // 2. 对于10Hz的激光雷达，ApproximateTime策略会自动处理时间同步
    //    队列越大，能容忍的时间差越大，不会因为时间差稍大就丢弃消息
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(50), *sub_front_, *sub_mid_, *sub_left_, *sub_right_, *sub_back_);
    
    // 初始化性能监控
    last_publish_time_ = std::chrono::high_resolution_clock::now();


    sync_->registerCallback(
        std::bind(&MultiLidarSplicing::callback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5));

    rclcpp::spin(shared_from_this());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLidarSplicing::processLidarCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
    const Lidar &lidar)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    fromROSMsg(*msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, lidar.getExtrinsic());
    return cloud;
}

void MultiLidarSplicing::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_front_msg,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_mid_msg,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_left_msg,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_right_msg,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_back_msg)
{
    auto startTime = std::chrono::high_resolution_clock::now();
    total_frames_++;

    // 顺序处理，避免每帧创建/销毁线程带来的抖动
    auto lidar_front_cloud = processLidarCloud(lidar_front_msg, lidar_front_);
    auto lidar_mid_cloud   = processLidarCloud(lidar_mid_msg,  lidar_mid_);
    auto lidar_left_cloud  = processLidarCloud(lidar_left_msg, lidar_left_);
    auto lidar_right_cloud = processLidarCloud(lidar_right_msg, lidar_right_);
    auto lidar_back_cloud  = processLidarCloud(lidar_back_msg,  lidar_back_);

    // 优化点云合并：预先计算总点数，一次性分配内存
    size_t total_points = lidar_front_cloud->size() + lidar_mid_cloud->size() + 
                          lidar_left_cloud->size() + lidar_right_cloud->size() + 
                          lidar_back_cloud->size();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    point_cloud->points.reserve(total_points);
    point_cloud->width = total_points;
    point_cloud->height = 1;
    point_cloud->is_dense = lidar_front_cloud->is_dense && lidar_mid_cloud->is_dense && 
                           lidar_left_cloud->is_dense && lidar_right_cloud->is_dense && 
                           lidar_back_cloud->is_dense;

    // 使用insert而不是+=，减少内存重新分配
    point_cloud->points.insert(point_cloud->points.end(), lidar_front_cloud->points.begin(), lidar_front_cloud->points.end());
    point_cloud->points.insert(point_cloud->points.end(), lidar_mid_cloud->points.begin(), lidar_mid_cloud->points.end());
    point_cloud->points.insert(point_cloud->points.end(), lidar_left_cloud->points.begin(), lidar_left_cloud->points.end());
    point_cloud->points.insert(point_cloud->points.end(), lidar_right_cloud->points.begin(), lidar_right_cloud->points.end());
    point_cloud->points.insert(point_cloud->points.end(), lidar_back_cloud->points.begin(), lidar_back_cloud->points.end());

    auto after_merge = std::chrono::high_resolution_clock::now();
    // 优化：直接构建ROS消息，避免pcl::toROSMsg的额外开销
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud, point_cloud_msg);
    point_cloud_msg.header = lidar_front_msg->header;
    point_cloud_msg.header.frame_id = frame_id_;
    
    // 发布消息
    point_cloud_publisher_->publish(point_cloud_msg);
    last_publish_time_ = std::chrono::high_resolution_clock::now();

    auto endTime = std::chrono::high_resolution_clock::now();
    double sec = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count() / 1e9;
    double merge_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(after_merge - startTime).count() / 1e9;
    double fps = 1.0 / sec;
    
    // 每10帧输出一次性能信息，减少日志开销
    // if (total_frames_ % 10 == 0) {
    //     double drop_rate = total_frames_ > 0 ? dropped_frames_ * 100.0 / total_frames_ : 0;
    //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //         "[帧 %zu] 处理: %.2fms | 理论FPS: %.1f | 点云: %zu | 丢弃: %zu (%.1f%%)",
    //         total_frames_, sec * 1000, fps, total_points, dropped_frames_, drop_rate);
    // }
    
    // 警告：如果处理时间超过100ms（10Hz周期），输出警告，并指明主要耗时段
    if (sec > 0.1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "处理时间 %.2fms (拼接 %.2fms) 超过10Hz周期(100ms)！", sec * 1000, merge_sec * 1000);
    }
}
