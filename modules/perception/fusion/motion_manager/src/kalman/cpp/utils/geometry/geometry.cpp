#include "utils/geometry/geometry.hpp"
namespace utils
{

double calcAzimuthAngle(
  const tf2_geometry_msgs::Point & p_from, const tf2_geometry_msgs::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

double calcElevationAngle(
  const tf2_geometry_msgs::Point & p_from, const tf2_geometry_msgs::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

tf2_geometry_msgs::Pose calcOffsetPose(
  const tf2_geometry_msgs::Pose & p, const double x, const double y, const double z,
  const double yaw)
{
  tf2_geometry_msgs::Pose pose;
  tf2_geometry_msgs::Transform transform;
  transform.translation = createTranslation(x, y, z);
//   transform.rotation = createQuaternionFromYaw(yaw);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  transform.rotation.x = q.getX();
  transform.rotation.y = q.getY();
  transform.rotation.z = q.getZ();
  transform.rotation.w = q.getW();
  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::transformMsgToTF(transform, tf_offset);
  tf2::poseMsgToTF(p, tf_pose);
//   tf2::toMsg(tf_pose * tf_offset, pose);
  tf2::Transform transform_tmp = tf_pose * tf_offset;
  tf2::poseTFToMsg(transform_tmp, pose);
  // pose.position.x = transform_tmp.getOrigin().getX();
  // pose.position.y = transform_tmp.getOrigin().getY();
  // pose.position.z = transform_tmp.getOrigin().getZ();
  // pose.orientation.x = transform_tmp.getRotation().getX();
  // pose.orientation.y = transform_tmp.getRotation().getY();
  // pose.orientation.z = transform_tmp.getRotation().getZ();
  // pose.orientation.w = transform_tmp.getRotation().getW();
  return pose;
}

tf2_geometry_msgs::Vector3 createTranslation(const double x, const double y, const double z)
{
  tf2_geometry_msgs::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}
// tf2_geometry_msgs::Quaternion createQuaternionFromYaw(const double yaw)
// {
//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   return tf2::toMsg(q);
// }

// tf2_geometry_msgs::Pose& toMsg(const tf2::Transform& in, tf2_geometry_msgs::Pose& out)
// {
//   toMsg(in.getOrigin(), out.position);
//   out.orientation = toMsg(in.getRotation());
//   return out;
// }

}