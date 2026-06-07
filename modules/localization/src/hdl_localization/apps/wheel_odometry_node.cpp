#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ros2_interface/msg/chassis.hpp>

class WheelOdometryNode : public rclcpp::Node {
public:
  WheelOdometryNode()
      : Node("wheel_odometry_node"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    sub_chassis_topic_ = declare_parameter<std::string>("sub_chassis_topic", "/chassis");
    sub_imu_topic_ = declare_parameter<std::string>("sub_imu_topic", "/livox/imu");
    pub_odom_topic_ = declare_parameter<std::string>("pub_odom_topic", "/odom_wheel");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "livox_frame");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    use_gps_angle_ = declare_parameter<bool>("use_gps_angle", false);

    chassis_sub_ = create_subscription<ros2_interface::msg::Chassis>(
        sub_chassis_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryNode::chassis_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        sub_imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryNode::imu_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic_, 10);
  }

private:
  void chassis_callback(const ros2_interface::msg::Chassis::SharedPtr msg) {
    current_speed_ = (msg->gear_location == 2) ? -msg->speed_mps : msg->speed_mps;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
    publish_time_ = msg->header.stamp;
    if (!initialized_) {
      start_theta_ = current_yaw_;
      last_theta_ = current_yaw_;
      last_time_ = publish_time_;
      initialized_ = true;
      return;
    }

    const rclcpp::Time current_time(publish_time_);
    const double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    theta_ = use_gps_angle_ ? current_yaw_ : current_yaw_ - start_theta_;
    const double delta_theta = theta_ - last_theta_;
    const double delta_distance = current_speed_ * dt;

    x_ += delta_distance * std::cos(theta_);
    y_ += delta_distance * std::sin(theta_);

    tf2::Quaternion odom_q;
    odom_q.setRPY(0.0, 0.0, theta_);
    odom_q.normalize();

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = publish_time_;
      transform.header.frame_id = frame_id_;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = tf2::toMsg(odom_q);
      tf_broadcaster_->sendTransform(transform);
    }

    if (publish_odom_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = publish_time_;
      odom.header.frame_id = frame_id_;
      odom.child_frame_id = child_frame_id_;
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf2::toMsg(odom_q);
      odom.twist.twist.linear.x = current_speed_ * std::cos(theta_);
      odom.twist.twist.linear.y = current_speed_ * std::sin(theta_);
      odom.twist.twist.angular.z = delta_theta / dt;
      odom_pub_->publish(odom);
    }

    last_theta_ = theta_;
    last_time_ = current_time;
  }

  std::string sub_chassis_topic_;
  std::string sub_imu_topic_;
  std::string pub_odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_ = true;
  bool publish_odom_ = true;
  bool use_gps_angle_ = false;

  bool initialized_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double last_theta_ = 0.0;
  double current_yaw_ = 0.0;
  double start_theta_ = 0.0;
  double current_speed_ = 0.0;
  builtin_interfaces::msg::Time publish_time_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<ros2_interface::msg::Chassis>::SharedPtr chassis_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
