#include "multi_lidar_splicing.h"
#include <stdexcept>
#include <algorithm>
#include <iomanip>

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
    size_t point_count = static_cast<size_t>(cloud_msg.width) * cloud_msg.height;
    cloud.points.resize(point_count);

    struct FieldInfo { int offset{-1}; uint8_t datatype{0}; };
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
    const size_t point_step = cloud_msg.point_step;
    const bool need_swap = cloud_msg.is_bigendian != system_is_bigendian;

    // Fast path: little-endian FLOAT32 (most common LiDAR format)
    bool fast_path = !need_swap &&
                     x_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                     y_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                     z_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                     (intensity_field.offset == -1 || intensity_field.datatype == sensor_msgs::msg::PointField::FLOAT32);

    if (fast_path) {
        for (size_t i = 0; i < point_count; ++i) {
            const uint8_t* ros_point = ros_data + i * point_step;
            auto& point = cloud[i];
            std::memcpy(&point.x, ros_point + x_field.offset, sizeof(float));
            std::memcpy(&point.y, ros_point + y_field.offset, sizeof(float));
            std::memcpy(&point.z, ros_point + z_field.offset, sizeof(float));
            if (intensity_field.offset != -1)
                std::memcpy(&point.intensity, ros_point + intensity_field.offset, sizeof(float));
            else
                point.intensity = 0.0f;
        }
    } else {
        auto parseFloat = [&](const uint8_t* data, const FieldInfo& field) -> float {
            if (!need_swap && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
                float v; std::memcpy(&v, data + field.offset, sizeof(float)); return v;
            }
            return readValue<float>(data, field.offset, field.datatype, cloud_msg.is_bigendian);
        };
        for (size_t i = 0; i < point_count; ++i) {
            const uint8_t* ros_point = ros_data + i * point_step;
            auto& point = cloud[i];
            point.x = parseFloat(ros_point, x_field);
            point.y = parseFloat(ros_point, y_field);
            point.z = parseFloat(ros_point, z_field);
            if (intensity_field.offset != -1)
                point.intensity = parseFloat(ros_point, intensity_field);
            else
                point.intensity = 0.0f;
        }
    }
}

MultiLidarSplicing::MultiLidarSplicing(const std::vector<LidarConfig> &lidar_configs,
                                       const std::string &frame_id,
                                       const std::string &publish_topic,
                                       const FilterRegion &filter_region,
                                       const FilterRegion &filter_region_outside,
                                       const SyncConfig &sync_config)
: rclcpp::Node("MultiLidarSplicing"),
  frame_id_(std::move(frame_id)),
  publish_topic_(std::move(publish_topic)),
  filter_region_(filter_region),
  filter_region_outside_(filter_region_outside),
  sync_config_(sync_config),
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
    
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, qos);

    msg_buffer_.resize(lidar_count_);
    last_published_stamps_.resize(lidar_count_, rclcpp::Time(0, 0, RCL_ROS_TIME));

    last_publish_time_ = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "MultiLidarSplicing initialized with %d lidars", lidar_count_);

    if (filter_region_.enable) {
        RCLCPP_INFO(this->get_logger(),
            "Filter region (inside) enabled: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
            filter_region_.x_min, filter_region_.x_max,
            filter_region_.y_min, filter_region_.y_max,
            filter_region_.z_min, filter_region_.z_max);
    }

    if (filter_region_outside_.enable) {
        RCLCPP_INFO(this->get_logger(),
            "Filter region (outside) enabled: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
            filter_region_outside_.x_min, filter_region_outside_.x_max,
            filter_region_outside_.y_min, filter_region_outside_.y_max,
            filter_region_outside_.z_min, filter_region_outside_.z_max);
    }
}

MultiLidarSplicing::~MultiLidarSplicing()
{
    stop_worker_ = true;
    task_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void MultiLidarSplicing::run()
{
    worker_thread_ = std::thread(&MultiLidarSplicing::workerLoop, this);

    rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
    sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sensor_qos.depth = 1;

    for (size_t i = 0; i < lidars_.size(); ++i) {
        std::string topic = lidars_[i].getChannel();
        auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, topic, sensor_qos);
        subscribers_[topic] = sub;
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic.c_str());
        
        size_t lidar_idx = i;
        sub->registerCallback([this, lidar_idx, topic](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Received message from %s, ---size: %ux%u", topic.c_str(), msg->width, msg->height);
            this->onLidarMessage(lidar_idx, msg);
        });
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to %zu lidar topics", subscribers_.size());

    sync_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { this->approximateTimeSync(); });

    rclcpp::spin(shared_from_this());
}

void MultiLidarSplicing::workerLoop()
{
    while (!stop_worker_) {
        std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> msgs;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            task_cv_.wait(lock, [this] { return !task_queue_.empty() || stop_worker_.load(); });
            if (stop_worker_) break;
            msgs = std::move(task_queue_.front());
            task_queue_.pop();
        }
        callback(msgs);
    }
}

void MultiLidarSplicing::approximateTimeSync()
{
    std::lock_guard<std::mutex> lock(msg_mutex_);
    tryPublishLocked();
}

void MultiLidarSplicing::onLidarMessage(size_t lidar_index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    rclcpp::Time msg_time(msg->header.stamp);
    RCLCPP_INFO(this->get_logger(), "Received message from lidar[%zu], stamp: %u.%09us, size: %ux%u",
                lidar_index, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);

    {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        auto& dq = msg_buffer_[lidar_index];
        dq.push_back({msg, msg_time, this->now()});
        if (dq.size() > MAX_BUFFER_PER_LIDAR) {
            dq.pop_front();
        }
        tryPublishLocked();
    }
}

void MultiLidarSplicing::tryPublishLocked()
{
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> synchronized_msgs(lidar_count_, nullptr);

    if (sync_config_.enable_approximate_time_sync) {
        rclcpp::Time latest_time(0, 0, RCL_ROS_TIME);
        for (size_t i = 0; i < msg_buffer_.size(); ++i) {
            if (!msg_buffer_[i].empty() && msg_buffer_[i].back().timestamp > latest_time) {
                latest_time = msg_buffer_[i].back().timestamp;
            }
        }

        if (latest_time.nanoseconds() != 0) {
            double tolerance = sync_config_.sync_time_tolerance;
            rclcpp::Time window_start(
                static_cast<int64_t>((latest_time.seconds() - tolerance) * 1e9), RCL_ROS_TIME);
            rclcpp::Time window_end(
                static_cast<int64_t>((latest_time.seconds() + tolerance) * 1e9), RCL_ROS_TIME);

            for (size_t i = 0; i < msg_buffer_.size(); ++i) {
                for (const auto& m : msg_buffer_[i]) {
                    if (m.timestamp >= window_start && m.timestamp <= window_end) {
                        if (synchronized_msgs[i] == nullptr ||
                            m.timestamp > rclcpp::Time(synchronized_msgs[i]->header.stamp)) {
                            synchronized_msgs[i] = m.msg;
                        }
                    }
                }
            }
        }
    } else {
        for (size_t i = 0; i < msg_buffer_.size(); ++i) {
            if (!msg_buffer_[i].empty()) {
                synchronized_msgs[i] = msg_buffer_[i].back().msg;
            }
        }
    }

    int valid_count = 0;
    for (size_t i = 0; i < synchronized_msgs.size(); ++i) {
        if (synchronized_msgs[i] != nullptr) {
            valid_count++;
        }
    }

    bool should_publish = false;
    if (sync_config_.strict_sync) {
        should_publish = (valid_count == lidar_count_);
    } else {
        should_publish = (valid_count >= sync_config_.min_lidars_for_fusion);
    }

    if (!should_publish) {
        return;
    }

    // 避免重复发布
    bool has_new_data = false;
    for (size_t i = 0; i < synchronized_msgs.size(); ++i) {
        if (synchronized_msgs[i] != nullptr) {
            rclcpp::Time ts(synchronized_msgs[i]->header.stamp);
            if (ts != last_published_stamps_[i]) {
                has_new_data = true;
                break;
            }
        }
    }
    if (!has_new_data) {
        return;
    }

    // 记录本次发布时间戳
    for (size_t i = 0; i < synchronized_msgs.size(); ++i) {
        if (synchronized_msgs[i] != nullptr) {
            last_published_stamps_[i] = rclcpp::Time(synchronized_msgs[i]->header.stamp);
        }
    }

    // 清空所有缓存
    for (auto& dq : msg_buffer_) {
        dq.clear();
    }

    // 推入工作线程队列
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.push(std::move(synchronized_msgs));
    }
    task_cv_.notify_one();
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
    if (!filter_region_.enable && !filter_region_outside_.enable) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    filtered->reserve(cloud->size());
    filtered->header = cloud->header;
    filtered->width = cloud->width;
    filtered->height = cloud->height;
    filtered->is_dense = false;

    for (const auto& point : cloud->points) {
        bool keep_point = true;

        if (filter_region_.enable) {
            if (point.x >= filter_region_.x_min && point.x <= filter_region_.x_max &&
                point.y >= filter_region_.y_min && point.y <= filter_region_.y_max &&
                point.z >= filter_region_.z_min && point.z <= filter_region_.z_max)
            {
                keep_point = false;
            }
        }

        if (filter_region_outside_.enable) {
            if (point.x < filter_region_outside_.x_min || point.x > filter_region_outside_.x_max ||
                point.y < filter_region_outside_.y_min || point.y > filter_region_outside_.y_max ||
                point.z < filter_region_outside_.z_min || point.z > filter_region_outside_.z_max)
            {
                keep_point = false;
            }
        }

        if (keep_point) {
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
    int valid_count = 0;
    for (size_t i = 0; i < msgs.size(); ++i) {
        if (msgs[i] != nullptr) {
            valid_count++;
        }
    }

    if (valid_count == 0) {
        return;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> processed_clouds;
    for (size_t i = 0; i < msgs.size(); ++i) {
        if (msgs[i] != nullptr) {
            processed_clouds.push_back(processLidarCloud(msgs[i], lidars_[i]));
        }
    }

    if (processed_clouds.empty()) {
        return;
    }

    auto merged_cloud = mergeClouds(processed_clouds);
    auto filtered_cloud = filterCloud(merged_cloud);

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, point_cloud_msg);

    // 使用参与拼接的雷达消息中最新的时间戳
    rclcpp::Time publish_stamp(0, 0, RCL_ROS_TIME);
    for (size_t i = 0; i < msgs.size(); ++i) {
        if (msgs[i] != nullptr) {
            rclcpp::Time ts(msgs[i]->header.stamp);
            if (ts > publish_stamp) {
                publish_stamp = ts;
            }
        }
    }
    if (publish_stamp.nanoseconds() == 0) {
        publish_stamp = this->now();
    }
    point_cloud_msg.header.stamp = publish_stamp;
    point_cloud_msg.header.frame_id = frame_id_;
    
    point_cloud_publisher_->publish(point_cloud_msg);
    last_publish_time_ = std::chrono::high_resolution_clock::now();

    // 打印延时信息
    rclcpp::Time actual_publish_time = this->now();
    int64_t pub_sec = actual_publish_time.seconds();
    int64_t pub_nanosec = actual_publish_time.nanoseconds() % 1000000000;
    std::ostringstream oss;
    oss << "Publish: " << pub_sec << "." << std::setw(9) << std::setfill('0') << pub_nanosec
        << ", Lidar: [";
    for (size_t i = 0; i < msgs.size(); ++i) {
        if (msgs[i] != nullptr) {
            oss << msgs[i]->header.stamp.sec << "." << std::setw(9) << std::setfill('0') << msgs[i]->header.stamp.nanosec;
        } else {
            oss << "null";
        }
        if (i < msgs.size() - 1) oss << ", ";
    }
    oss << "], Delay: ";
    for (const auto& m : msgs) {
        if (m != nullptr) {
            rclcpp::Time lidar_time(m->header.stamp);
            double delay_sec = (actual_publish_time - lidar_time).seconds();
            oss << (delay_sec * 1000.0) << "ms";
            break;
        }
    }
    oss << ", Proc: " << (std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - last_publish_time_).count()) << "ms";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}
