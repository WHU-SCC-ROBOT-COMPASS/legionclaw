/*
 * @FilePath: /modules/prediction/src/common/transformer.hpp
 * @Descripttion:
 * @version:
 * @Author: sunjay sunjay.lee@qq.com
 * @Date: 2022-08-16 14:08:14
 * @LastEditTime: 2022-09-06 17:26:01
 */
#pragma once
#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace legionclaw
{
    namespace prediction
    {
        namespace transform
        {
            class Transformer
            {
            private:
                //旋转矩阵
                Eigen::Matrix3d rotation_matrix_;
                //平移向量
                Eigen::Vector3d translation_vector_;

            public:
                Transformer() = default;
                ~Transformer() = default;
                //@brief 设置旋转矩阵
                inline void set_rotation_matrix(Eigen::Matrix3d rotation_matrix)
                {
                    rotation_matrix_ = rotation_matrix;
                }
                //@brief 获取旋转矩阵
                inline Eigen::Matrix3d get_rotation_matrix()
                {
                    return rotation_matrix_;
                }

                //@brief 设置四元数
                inline void set_quaternion(Eigen::Quaterniond quaternion)
                {
                    rotation_matrix_ = quaternion.toRotationMatrix();
                }
                //@brief 获取四元数
                inline Eigen::Quaterniond get_quaternion()
                {
                    Eigen::Quaterniond quaternion(rotation_matrix_);
                    return quaternion;
                }

                //@brief 设置欧拉角
                inline void set_rpy(Eigen::Vector3d rpy)
                {
                    Eigen::AngleAxisd roll = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitch = Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yaw = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
                    rotation_matrix_ = yaw * pitch * roll;
                }
                //@brief 获取欧拉角
                inline Eigen::Vector3d get_rpy()
                {
                    Eigen::Vector3d rpy = rotation_matrix_.eulerAngles(0, 1, 0);
                    return rpy;
                }

                //@brief 获取航向角
                inline double get_yaw()
                {
                    Eigen::Vector3d rpy = rotation_matrix_.eulerAngles(0, 1, 0);
                    return rpy.y();
                }

                //@brief 设置平移向量
                inline void set_translation(Eigen::Vector3d xyz)
                {
                    translation_vector_ = xyz;
                }

                //@brief 清除
                inline void clear()
                {
                    rotation_matrix_ = Eigen::Matrix3d::Zero();
                    translation_vector_ = Eigen::Vector3d::Zero();
                }

                //@brief 坐标转换
                inline Eigen::Vector3d trans(Eigen::Vector3d input)
                {
                    Eigen::Vector3d out = rotation_matrix_ * input + translation_vector_;
                    return out;
                }

                //@brief 坐标转换
                inline Eigen::Vector3d trans_inverse(Eigen::Vector3d input)
                {
                    Eigen::Vector3d out = rotation_matrix_.inverse() * (input - translation_vector_);
                    return out;
                }

                //@brief 坐标转换
                inline Eigen::Vector3d rot(Eigen::Vector3d input)
                {
                    Eigen::Vector3d out = rotation_matrix_ * input;
                    return out;
                }

                //@brief 坐标转换
                inline Eigen::Vector3d rot_inverse(Eigen::Vector3d input)
                {
                    Eigen::Vector3d out = rotation_matrix_.inverse() * input;
                    return out;
                }
            };

        } // namespace transform

    } // namespace prediction

} // namespace legionclaw
