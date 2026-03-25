// NOTE:
// This utility header has been refactored to remove direct ROS2 dependencies.
// It now provides helpers to convert between Eigen matrices and the common
// interface point cloud type `legionclaw::interface::PointCloud`.

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "modules/common/interface/point_cloud.hpp"

namespace patchworkpp_ros::utils {

// 将通用 PointCloud 转为 Eigen::MatrixXf（列: x, y, z, intensity）
inline Eigen::MatrixXf PointCloud2ToEigenMat(
    const legionclaw::interface::PointCloud &cloud) {
  std::vector<legionclaw::interface::PointXYZIRT> points_vec;
  cloud.point(points_vec);

  const std::size_t num_points = points_vec.size();
  Eigen::MatrixXf mat(num_points, 4);

  for (std::size_t i = 0; i < num_points; ++i) {
    const auto &p = points_vec[i];
    mat(static_cast<Eigen::Index>(i), 0) =
        static_cast<float>(p.x());
    mat(static_cast<Eigen::Index>(i), 1) =
        static_cast<float>(p.y());
    mat(static_cast<Eigen::Index>(i), 2) =
        static_cast<float>(p.z());
    mat(static_cast<Eigen::Index>(i), 3) =
        static_cast<float>(p.intensity());
  }

  return mat;
}

// 将 Eigen::MatrixX3f 转为通用 PointCloud
inline legionclaw::interface::PointCloud EigenMatToPointCloud2(
    const Eigen::MatrixX3f &points,
    const legionclaw::interface::Header &header) {
  legionclaw::interface::PointCloud cloud;
  cloud.set_header(header);
  cloud.set_frame_id(header.frame_id());
  cloud.set_is_dense(true);

  const std::size_t num_points =
      static_cast<std::size_t>(points.rows());

  std::vector<legionclaw::interface::PointXYZIRT> pts;
  pts.reserve(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    legionclaw::interface::PointXYZIRT p;
    p.set_x(points(static_cast<Eigen::Index>(i), 0));
    p.set_y(points(static_cast<Eigen::Index>(i), 1));
    p.set_z(points(static_cast<Eigen::Index>(i), 2));
    p.set_intensity(0);  // 此处暂不使用强度信息
    pts.emplace_back(p);
  }

  cloud.set_point(pts);
  cloud.set_width(static_cast<uint32_t>(num_points));
  cloud.set_height(1);

  return cloud;
}

}  // namespace patchworkpp_ros::utils
