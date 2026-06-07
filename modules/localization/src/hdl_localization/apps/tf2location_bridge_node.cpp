#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <ros2_interface/msg/chassis.hpp>
#include <ros2_interface/msg/location.hpp>
#include <ros2_interface/msg/odometry.hpp>

#include <hdl_localization/projector.hpp>

class Tf2LocationBridgeNode : public rclcpp::Node {
public:
  Tf2LocationBridgeNode()
      : Node("tf2location_bridge"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    target_frame_ = declare_parameter<std::string>("target_frame", "livox_frame");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    location_topic_ = declare_parameter<std::string>("location_topic", "/localization/slam/Location");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/localization/slam/Odometry");
    status_topic_ = declare_parameter<std::string>("status_topic", "/status");
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/odom_wheel");
    nav_odom_topic_ = declare_parameter<std::string>("nav_odom_topic", "/odom");
    chassis_topic_ = declare_parameter<std::string>("chassis_topic", "/chassis");
    use_rtk_flag_ = declare_parameter<bool>("use_rtk_flag", false);
    origin_lat_ = declare_parameter<double>("origin_lat", 0.0);
    origin_lon_ = declare_parameter<double>("origin_lon", 0.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    monitor_time_ = declare_parameter<double>("monitor_time", 1.0);
    trans_error_ = declare_parameter<double>("trans_error", 2.0);
    enable_monitor_ = declare_parameter<bool>("enable_monitor", true);
    auxiliary_type_ = declare_parameter<int>("auxiliary_type", 2);
    odom_type_default_ = declare_parameter<int>("odom_type_default", 3);
    odom_type_with_wheel_ = declare_parameter<int>("odom_type_with_wheel", 1);

    if (std::abs(origin_lat_) > 1e-9 || std::abs(origin_lon_) > 1e-9) {
      projector_.set_origin_ll(origin_lat_, origin_lon_);
    }

    location_pub_ = create_publisher<ros2_interface::msg::Location>(location_topic_, 10);
    odometry_pub_ = create_publisher<ros2_interface::msg::Odometry>(odometry_topic_, 10);

    status_sub_ = create_subscription<hdl_localization::msg::ScanMatchingStatus>(
        status_topic_, 10, std::bind(&Tf2LocationBridgeNode::status_callback, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_odom_topic_, 10, std::bind(&Tf2LocationBridgeNode::wheel_odom_callback, this, std::placeholders::_1));
    nav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        nav_odom_topic_, 10, std::bind(&Tf2LocationBridgeNode::nav_odom_callback, this, std::placeholders::_1));
    chassis_sub_ = create_subscription<ros2_interface::msg::Chassis>(
        chassis_topic_, rclcpp::SensorDataQoS(),
        std::bind(&Tf2LocationBridgeNode::chassis_callback, this, std::placeholders::_1));

    set_check_flag_srv_ = create_service<std_srvs::srv::SetBool>(
        "/set_check_flag",
        std::bind(&Tf2LocationBridgeNode::set_check_flag, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0)),
        std::bind(&Tf2LocationBridgeNode::publish_messages, this));
  }

private:
  void status_callback(const hdl_localization::msg::ScanMatchingStatus::SharedPtr msg) {
    rtk_flag_ = msg->has_converged ? 1 : 3;
    has_status_ = true;
  }

  void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_wheel_odom_ = *msg;
    last_wheel_odom_time_ = msg->header.stamp;
  }

  void nav_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_nav_odom_ = *msg;
    last_nav_odom_time_ = msg->header.stamp;
  }

  void chassis_callback(const ros2_interface::msg::Chassis::SharedPtr msg) {
    latest_chassis_ = *msg;
    has_chassis_ = true;
  }

  void set_check_flag(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    enable_monitor_ = request->data;
    response->success = true;
    response->message = enable_monitor_ ? "monitor enabled" : "monitor disabled";
  }

  bool recent(const builtin_interfaces::msg::Time& stamp, double seconds) const {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return false;
    }
    const auto age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age < seconds;
  }

  const nav_msgs::msg::Odometry* select_velocity_source() const {
    if (recent(last_wheel_odom_time_, 1.0)) {
      return &latest_wheel_odom_;
    }
    if (recent(last_nav_odom_time_, 1.0)) {
      return &latest_nav_odom_;
    }
    return nullptr;
  }

  void fill_custom_odometry(const geometry_msgs::msg::TransformStamped& transform) {
    ros2_interface::msg::Odometry odom_msg;
    odom_msg.header = transform.header;
    odom_msg.position.x = transform.transform.translation.x;
    odom_msg.position.y = transform.transform.translation.y;
    odom_msg.position.z = transform.transform.translation.z;
    odom_msg.orientation.qx = transform.transform.rotation.x;
    odom_msg.orientation.qy = transform.transform.rotation.y;
    odom_msg.orientation.qz = transform.transform.rotation.z;
    odom_msg.orientation.qw = transform.transform.rotation.w;

    const auto* source = select_velocity_source();
    if (source) {
      odom_msg.covariance.assign(source->pose.covariance.begin(), source->pose.covariance.end());
    } else {
      odom_msg.covariance.assign(36, 0.0);
    }

    odometry_pub_->publish(odom_msg);
  }

  void update_monitor_flag() {
    if (!enable_monitor_ || monitor_time_ <= 0.0) {
      return;
    }

    const rclcpp::Time current_time = now();
    const rclcpp::Time past_time = current_time - rclcpp::Duration::from_seconds(monitor_time_);

    try {
      const auto map_delta = tf_buffer_.lookupTransform(target_frame_, past_time, target_frame_, current_time, map_frame_);
      const auto odom_delta = tf_buffer_.lookupTransform(target_frame_, past_time, target_frame_, current_time, odom_frame_);

      const double dx = map_delta.transform.translation.x - odom_delta.transform.translation.x;
      const double dy = map_delta.transform.translation.y - odom_delta.transform.translation.y;
      const double position_error = std::sqrt(dx * dx + dy * dy);

      if (position_error > trans_error_) {
        rtk_flag_ = 3;
        enable_monitor_ = false;
      } else if (!use_rtk_flag_ || !has_status_) {
        rtk_flag_ = 1;
      }
    } catch (const tf2::TransformException&) {
    }
  }

  void publish_messages() {
    const auto frames_yaml = tf_buffer_.allFramesAsYAML();
    if (frames_yaml.find(map_frame_) == std::string::npos || frames_yaml.find(target_frame_) == std::string::npos) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, target_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "lookupTransform %s->%s failed: %s",
                           map_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    update_monitor_flag();

    tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y,
                      transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);

    ros2_interface::msg::Location location_msg;
    location_msg.header = transform.header;
    location_msg.header.frame_id = map_frame_;
    location_msg.roll = roll;
    location_msg.pitch = pitch;
    location_msg.heading = yaw;
    location_msg.origin_lat = origin_lat_;
    location_msg.origin_lon = origin_lon_;
    location_msg.utm_position.x = transform.transform.translation.x;
    location_msg.utm_position.y = transform.transform.translation.y;
    location_msg.utm_position.z = transform.transform.translation.z;
    location_msg.rtk_flag = use_rtk_flag_ ? rtk_flag_ : 1;
    location_msg.auxiliary_type = auxiliary_type_;
    location_msg.odom_type = recent(last_wheel_odom_time_, 1.0) ? odom_type_with_wheel_ : odom_type_default_;
    location_msg.location_valid_flag = location_msg.rtk_flag == 3 ? 0 : 1;
    location_msg.change_origin_flag = 0;

    if (projector_.initialized()) {
      const auto ll = projector_.reverse(location_msg.utm_position.x, location_msg.utm_position.y);
      location_msg.position.lat = ll.lat;
      location_msg.position.lon = ll.lon;
      location_msg.position.height = location_msg.utm_position.z;
    }

    const auto* velocity_source = select_velocity_source();
    if (velocity_source) {
      location_msg.linear_velocity.x = velocity_source->twist.twist.linear.x;
      location_msg.linear_velocity.y = velocity_source->twist.twist.linear.y;
      location_msg.linear_velocity.z = velocity_source->twist.twist.linear.z;
      location_msg.angular_velocity.x = velocity_source->twist.twist.angular.x;
      location_msg.angular_velocity.y = velocity_source->twist.twist.angular.y;
      location_msg.angular_velocity.z = velocity_source->twist.twist.angular.z;
    } else if (last_publish_time_.seconds() > 0.0) {
      const double dt = (rclcpp::Time(transform.header.stamp) - last_publish_time_).seconds();
      if (dt > 0.0) {
        double delta_yaw = yaw - last_heading_;
        delta_yaw = std::fmod(delta_yaw, 2.0 * M_PI);
        if (delta_yaw > M_PI) {
          delta_yaw -= 2.0 * M_PI;
        } else if (delta_yaw < -M_PI) {
          delta_yaw += 2.0 * M_PI;
        }
        location_msg.angular_velocity.z = delta_yaw / dt;
      }
    }

    if (has_chassis_ && velocity_source == nullptr) {
      location_msg.linear_velocity.x = latest_chassis_.speed_mps;
    }

    location_pub_->publish(location_msg);
    fill_custom_odometry(transform);

    last_publish_time_ = rclcpp::Time(transform.header.stamp);
    last_heading_ = yaw;
  }

  std::string map_frame_;
  std::string target_frame_;
  std::string odom_frame_;
  std::string location_topic_;
  std::string odometry_topic_;
  std::string status_topic_;
  std::string wheel_odom_topic_;
  std::string nav_odom_topic_;
  std::string chassis_topic_;
  bool use_rtk_flag_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double publish_rate_ = 20.0;
  double monitor_time_ = 1.0;
  double trans_error_ = 2.0;
  bool enable_monitor_ = true;
  int auxiliary_type_ = 2;
  int odom_type_default_ = 3;
  int odom_type_with_wheel_ = 1;
  int rtk_flag_ = 1;
  bool has_status_ = false;
  bool has_chassis_ = false;
  double last_heading_ = 0.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  hdl_localization::projector::Projector projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Odometry latest_wheel_odom_;
  nav_msgs::msg::Odometry latest_nav_odom_;
  ros2_interface::msg::Chassis latest_chassis_;
  builtin_interfaces::msg::Time last_wheel_odom_time_;
  builtin_interfaces::msg::Time last_nav_odom_time_;

  rclcpp::Publisher<ros2_interface::msg::Location>::SharedPtr location_pub_;
  rclcpp::Publisher<ros2_interface::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Subscription<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_odom_sub_;
  rclcpp::Subscription<ros2_interface::msg::Chassis>::SharedPtr chassis_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_check_flag_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tf2LocationBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
