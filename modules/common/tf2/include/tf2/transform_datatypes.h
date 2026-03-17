/*
 * @Author: duanchengwen
 * @Date: 2021-05-24 20:48:18
 * @LastEditors: duanchengwen
 * @LastEditTime: 2021-06-08 16:51:50
 * @Description: file content
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef TF2_TRANSFORM_DATATYPES_H
#define TF2_TRANSFORM_DATATYPES_H

#include <string>
#include "time.h"
#include "LinearMath/Vector3.h"
#include "LinearMath/Transform.h"
#include "LinearMath/Quaternion.h"
#include "../geometry_msgs/transform_stamped.h"

namespace tf2 {

typedef tf2::Vector3 Point;
typedef tf2::Transform Pose;

static const double QUATERNION_TOLERANCE = 0.1f;

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf2 datatype equivilant of a MessageStamped */
template <typename T> class Stamped : public T {
public:
  Time stamp_;           ///< The timestamp associated with this data
  std::string frame_id_; ///< The frame_id associated this data

  /** Default constructor */
  Stamped()
      : frame_id_(
            "NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){}; // Default constructor used
                                                     // only for preallocation

  /** Full constructor */
  Stamped(const T& input, const Time& timestamp, const std::string& frame_id)
      : T(input), stamp_(timestamp), frame_id_(frame_id){};

  /** Copy Constructor */
  Stamped(const Stamped<T>& s)
      : T(s), stamp_(s.stamp_), frame_id_(s.frame_id_) {}

  /** Set the data element */
  void setData(const T& input) { *static_cast<T*>(this) = input; };
};

/** \brief Comparison Operator for Stamped datatypes */
template <typename T>
bool operator==(const Stamped<T>& a, const Stamped<T>& b) {
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
         static_cast<const T&>(a) == static_cast<const T&>(b);
};

/** \brief The Stamped Transform datatype used by tf2 */
class StampedTransform : public tf2::Transform {
public:
  Time stamp_;           ///< The timestamp associated with this transform
  std::string frame_id_; ///< The frame_id of the coordinate frame  in which
                         ///< this transform is defined
  std::string child_frame_id_; ///< The frame_id of the coordinate frame this
                               ///< transform defines
  StampedTransform(const tf2::Transform& input, const Time& timestamp,
                   const std::string& frame_id,
                   const std::string& child_frame_id)
      : tf2::Transform(input), stamp_(timestamp), frame_id_(frame_id),
        child_frame_id_(child_frame_id) {}

  /** \brief Default constructor only to be used for preallocation */
  StampedTransform(){};

  /** \brief Set the inherited Traonsform data */
  void setData(const tf2::Transform& input) {
    *static_cast<tf2::Transform*>(this) = input;
  };
};

/** \brief Comparison operator for StampedTransform */
static inline bool operator==(const StampedTransform& a,
                              const StampedTransform& b) {
  return a.frame_id_ == b.frame_id_ && a.child_frame_id_ == b.child_frame_id_ &&
         a.stamp_ == b.stamp_ &&
         static_cast<const tf2::Transform&>(a) ==
             static_cast<const tf2::Transform&>(b);
}

/** \brief construct a Quaternion from Fixed angles
 * \param roll The roll about the X axis
 * \param pitch The pitch about the Y axis
 * \param yaw The yaw about the Z axis
 * \return The quaternion constructed
 */
static inline Quaternion createQuaternionFromRPY(double roll, double pitch,
                                                 double yaw) {
  Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

/** \brief construct a Quaternion from yaw only
 * \param yaw The yaw about the Z axis
 * \return The quaternion constructed
 */
static inline Quaternion createQuaternionFromYaw(double yaw) {
  Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

/** \brief convert Quaternion msg to Quaternion */
static inline void quaternionMsgToTF(const tf2_geometry_msgs::Quaternion& msg,
                                     Quaternion& bt) {
  bt = Quaternion(msg.x, msg.y, msg.z, msg.w);
  if (fabs(bt.length2() - 1) > QUATERNION_TOLERANCE) {
    // ROS_WARN("MSG to TF: Quaternion Not Properly Normalized");
    bt.normalize();
  }
}

/** \brief convert Quaternion to Quaternion msg*/
static inline void quaternionTFToMsg(const Quaternion& bt,
                                     tf2_geometry_msgs::Quaternion& msg) {
  if (fabs(bt.length2() - 1) > QUATERNION_TOLERANCE) {
    // ROS_WARN("TF to MSG: Quaternion Not Properly Normalized");
    Quaternion bt_temp = bt;
    bt_temp.normalize();
    msg.x = bt_temp.x();
    msg.y = bt_temp.y();
    msg.z = bt_temp.z();
    msg.w = bt_temp.w();
  } else {
    msg.x = bt.x();
    msg.y = bt.y();
    msg.z = bt.z();
    msg.w = bt.w();
  }
}

/** \brief convert Vector3 to Vector3 msg*/
static inline void vector3TFToMsg(const Vector3& bt_v,
                                  tf2_geometry_msgs::Vector3& msg_v) {
  msg_v.x = bt_v.x();
  msg_v.y = bt_v.y();
  msg_v.z = bt_v.z();
}

/** \brief convert Transform msg to Transform */
static inline void transformMsgToTF(const tf2_geometry_msgs::Transform& msg,
                                    Transform& bt) {
  bt = Transform(
      Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z,
                 msg.rotation.w),
      Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}
/** \brief convert Transform to Transform msg*/
static inline void transformTFToMsg(const Transform& bt,
                                    tf2_geometry_msgs::Transform& msg) {
  vector3TFToMsg(bt.getOrigin(), msg.translation);
  quaternionTFToMsg(bt.getRotation(), msg.rotation);
}

/** \brief convert TransformStamped msg to tf::StampedTransform */
static inline void
transformStampedMsgToTF(const tf2_geometry_msgs::TransformStamped& msg,
                        StampedTransform& bt) {
  transformMsgToTF(msg.transform, bt);
  bt.stamp_ = msg.header.stamp;
  bt.frame_id_ = msg.header.frame_id;
  bt.child_frame_id_ = msg.child_frame_id;
}
/** \brief convert tf::StampedTransform to TransformStamped msg*/
static inline void
transformStampedTFToMsg(const StampedTransform& bt,
                        tf2_geometry_msgs::TransformStamped& msg) {
  transformTFToMsg(bt, msg.transform);
  msg.header.stamp = bt.stamp_;
  msg.header.frame_id = bt.frame_id_;
  msg.child_frame_id = bt.child_frame_id_;
}

/** \brief convert Point msg to Point */
static inline void pointMsgToTF(const tf2_geometry_msgs::Point& msg_v,
                                Point& bt_v) {
  bt_v = Vector3(msg_v.x, msg_v.y, msg_v.z);
}
/** \brief convert Point to Point msg*/
static inline void pointTFToMsg(const Point& bt_v,
                                tf2_geometry_msgs::Point& msg_v) {
  msg_v.x = bt_v.x();
  msg_v.y = bt_v.y();
  msg_v.z = bt_v.z();
}

/** \brief convert Pose msg to Pose */
static inline void poseMsgToTF(const tf2_geometry_msgs::Pose& msg, Pose& bt) {
  bt = Transform(Quaternion(msg.orientation.x, msg.orientation.y,
                            msg.orientation.z, msg.orientation.w),
                 Vector3(msg.position.x, msg.position.y, msg.position.z));
}
/** \brief convert Pose to Pose msg*/
static inline void poseTFToMsg(const Pose& bt, tf2_geometry_msgs::Pose& msg) {
  pointTFToMsg(bt.getOrigin(), msg.position);
  quaternionTFToMsg(bt.getRotation(), msg.orientation);
}

} // namespace tf2
#endif // TF2_TRANSFORM_DATATYPES_H
