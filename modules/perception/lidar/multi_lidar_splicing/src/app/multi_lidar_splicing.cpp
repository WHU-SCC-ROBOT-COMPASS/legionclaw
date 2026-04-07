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

static void fromROSMsg(const sensor_msgs::msg::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.header.stamp    = rclcpp::Time(cloud_msg.header.stamp).nanoseconds() / 1000;
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

struct BufferedMessage {
    size_t lidar_index;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    rclcpp::Time timestamp;
    BufferedMessage(size_t idx, sensor_msgs::msg::PointCloud2::ConstSharedPtr m, rclcpp::Time t)
        : lidar_index(idx), msg(std::move(m)), timestamp(t) {}
};

static std::mutex g_msg_mutex;
static std::vector<BufferedMessage> g_msg_buffer;
static constexpr double SYNC_TIME_TOLERANCE = 0.05;

MultiLidarSplicing::MultiLidarSplicing(const std::vector<LidarConfig> &lidar_configs,
                                       const std::string &frame_id,
                                       const std::string &publish_topic,
                                       const FilterRegion &filter_region)
: rclcpp::Node("MultiLidarSplicing"),
  frame_id_(std::move(frame_id)),
  publish_topic_(std::move(publish_topic)),
  filter_region_(filter_region),
  lidar_count_(static_cast<int>(lidar_configs.size()))
{
    for (size_t i = 0; i < lidar_configs.size(); ++i) {
        lidars_.push_back(Lidar(lidar_configs[i].path));
        lidar_name_to_index_[lidar_configs[i].name] = i;
    }

    bool use_sim_time = false;
    try {
        if (this->has_parameter("use_sim_time")) {
            use_sim_time = this->get_parameter("use_sim_time").as_bool();
        }
    } catch (const std::exception& e) {
        use_sim_time = false;
    }
    
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "使用仿真时间 (use_sim_time=true)");
    } else {
        RCLCPP_INFO(this->get_logger(), "使用系统时间 (use_sim_time=false)");
    }
    
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(100));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, qos);

    last_publish_time_ = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "MultiLidarSplicing initialized with %d lidars", lidar_count_);

    if (filter_region_.enable) {
        RCLCPP_INFO(this->get_logger(),
            "Filter region enabled: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
            filter_region_.x_min, filter_region_.x_max,
            filter_region_.y_min, filter_region_.y_max,
            filter_region_.z_min, filter_region_.z_max);
    }
}

MultiLidarSplicing::~MultiLidarSplicing() = default;

void MultiLidarSplicing::run()
{
    rmw_qos_profile_t reliable_sensor_qos = rmw_qos_profile_sensor_data;
    reliable_sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    reliable_sensor_qos.depth = 100;

    for (size_t i = 0; i < lidars_.size(); ++i) {
        std::string topic = lidars_[i].getChannel();
        auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, topic, reliable_sensor_qos);
        subscribers_[topic] = sub;
        
        size_t lidar_idx = i;
        sub->registerCallback([this, lidar_idx](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            this->onLidarMessage(lidar_idx, msg);
        });
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to %zu lidar topics", subscribers_.size());

    rclcpp::spin(shared_from_this());
}

void MultiLidarSplicing::onLidarMessage(size_t lidar_index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    rclcpp::Time msg_time(msg->header.stamp);
    
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> synchronized_msgs;
    
    {
        std::lock_guard<std::mutex> lock(g_msg_mutex);
        
        g_msg_buffer.emplace_back(lidar_index, msg, msg_time);
        
        auto now = this->now();
        g_msg_buffer.erase(
            std::remove_if(g_msg_buffer.begin(), g_msg_buffer.end(),
                [now](const BufferedMessage& m) {
                    return (now - m.timestamp).seconds() > 1.0;
                }),
            g_msg_buffer.end());
        
        if (g_msg_buffer.size() >= static_cast<size_t>(lidar_count_)) {
            rclcpp::Time sync_time(0, 0, RCL_ROS_TIME);
            for (const auto& m : g_msg_buffer) {
                if (m.timestamp > sync_time) {
                    sync_time = m.timestamp;
                }
            }
            
            synchronized_msgs.resize(lidar_count_, nullptr);
            std::vector<BufferedMessage> remaining;
            
            for (auto& buffered : g_msg_buffer) {
                if (std::abs((buffered.timestamp - sync_time).seconds()) < SYNC_TIME_TOLERANCE) {
                    if (buffered.lidar_index < synchronized_msgs.size() && synchronized_msgs[buffered.lidar_index] == nullptr) {
                        synchronized_msgs[buffered.lidar_index] = buffered.msg;
                    }
                } else {
                    remaining.push_back(std::move(buffered));
                }
            }
            
            bool all_received = true;
            for (const auto& msg_ptr : synchronized_msgs) {
                if (msg_ptr == nullptr) {
                    all_received = false;
                    break;
                }
            }
            
            if (all_received) {
                g_msg_buffer = std::move(remaining);
            } else {
                synchronized_msgs.clear();
            }
        }
    }
    
    if (!synchronized_msgs.empty()) {
        callback(synchronized_msgs);
    }
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

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLidarSplicing::mergeClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds)
{
    size_t total_points = 0;
    bool is_dense = true;

    for (const auto& cloud : clouds) {
        total_points += cloud->size();
        is_dense = is_dense && cloud->is_dense;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    result->points.reserve(total_points);
    result->width = total_points;
    result->height = 1;
    result->is_dense = is_dense;

    for (const auto& cloud : clouds) {
        result->points.insert(result->points.end(), cloud->points.begin(), cloud->points.end());
    }

    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLidarSplicing::filterCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    if (!filter_region_.enable) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    filtered->reserve(cloud->size());
    filtered->header = cloud->header;
    filtered->width = cloud->width;
    filtered->height = cloud->height;
    filtered->is_dense = false;

    for (const auto& point : cloud->points) {
        if (point.x < filter_region_.x_min || point.x > filter_region_.x_max ||
            point.y < filter_region_.y_min || point.y > filter_region_.y_max ||
            point.z < filter_region_.z_min || point.z > filter_region_.z_max)
        {
            filtered->points.push_back(point);
        }
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;

    return filtered;
}

void MultiLidarSplicing::callback(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &msgs)
{
    if (msgs.size() != static_cast<size_t>(lidar_count_)) {
        RCLCPP_WARN(this->get_logger(), "Received %zu messages but expected %d", msgs.size(), lidar_count_);
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    total_frames_++;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> processed_clouds;
    for (size_t i = 0; i < msgs.size(); ++i) {
        processed_clouds.push_back(processLidarCloud(msgs[i], lidars_[i]));
    }

    auto merged_cloud = mergeClouds(processed_clouds);

    auto filtered_cloud = filterCloud(merged_cloud);

    auto after_merge = std::chrono::high_resolution_clock::now();

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, point_cloud_msg);
    point_cloud_msg.header = msgs[0]->header;
    point_cloud_msg.header.frame_id = frame_id_;
    
    point_cloud_publisher_->publish(point_cloud_msg);
    last_publish_time_ = std::chrono::high_resolution_clock::now();

    auto endTime = std::chrono::high_resolution_clock::now();
    double sec = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count() / 1e9;
    double merge_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(after_merge - startTime).count() / 1e9;
    
    if (sec > 0.1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "处理时间 %.2fms (拼接 %.2fms) 超过10Hz周期(100ms)！", sec * 1000, merge_sec * 1000);
    }
}