/**
 * @file              math_tools.h
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-08-23 10:49:42
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#pragma once

#include <math.h>

#include <array>
#include <vector>

#include "modules/common/interface/path_point.hpp"
#include "modules/common/math/vec2d.h"

namespace legionclaw {
namespace common {
namespace math {
using legionclaw::common::math::Vec2d;
/**
 * @brief 角度转弧度。
 * @param d 输入量：角度。
 * @return 弧度。
 */
double D2R(double d);

/**
 * @brief 弧度转角度。
 * @param r 输入量：弧度。
 * @return 角度。
 */
double R2D(double r);

/**
 * @brief 计算从点(x1,y1)到点(x2,y2)的方向角（legion2.0）。
 * @param x1 输入量：起点的x坐标。
 * @param y1 输入量：起点的y坐标。
 * @param x2 输入量：终点的x坐标。
 * @param y2 输入量：终点的y坐标。
 * @return 从点(x1,y1)到点(x2,y2)的方向角（与y轴夹角，单位：°）。
 */
double GetHeading(double x1, double y1, double x2, double y2);

/**
 * @brief 计算地图中心线切线方向的航向角。
 * @param dx 输入量：dx。
 * @param dy 输入量：dy。
 * @return 某一点的切线方向角（与x轴夹角,正东方向逆时针，单位：rad）。
 */
double GetHeadingRadian(double dx, double dy);

/**
 * @class Transfer
 * @brief
 * 局部坐标系和全局坐标系之间的转换（角度为与x轴方向的夹角，单位：弧度）。
 */
class Transfer {
public:
  /**
   * @brief 构造函数
   */
  Transfer();
  /**
   * @brief 析构函数
   */
  virtual ~Transfer();

  /**
   * @brief 设置局部坐标系的原点坐标和x轴指向,
   * 以（x0,y0）为原点，angle为x轴指向，建立坐标系。
   * @param x0 输入量：局部坐标系原点x坐标（全局坐标系下）。
   * @param y0 输入量：局部坐标系原点y坐标（全局坐标系下）。
   * @param angle 输入量：局部坐标系x轴指向。
   */
  void SetOrigin(double x0, double y0, double angle);

  /**
   * @brief 将局部坐标系的坐标转换到全局坐标系下。
   * @param x_local 输入量：局部坐标系x坐标
   * @param y_local 输入量：局部坐标系y坐标
   * @param x_global 输出量：全局坐标系x坐标
   * @param y_global 输出量：全局坐标系y坐标
   */
  bool TransferL2G(double x_local, double y_local, double& x_global,
                   double& y_global);

  /**
   * @brief 将局部坐标系的坐标及方向角转换到全局坐标系下。
   * @param x_local 输入量：局部坐标系x坐标
   * @param y_local 输入量：局部坐标系y坐标
   * @param h_local 输入量：局部坐标系方向角
   * @param x_global 输出量：全局坐标系x坐标
   * @param y_global 输出量：全局坐标系y坐标
   * @param h_global 输出量：全局坐标系方向角
   */
  bool TransferL2G(double x_local, double y_local, double h_local,
                   double& x_global, double& y_global, double& h_global);

  /**
   * @brief 将局部坐标系的导航点坐标,航向角和曲率转换到全局坐标系下。
   * @param p_local 输入量：局部坐标系的导航点
   * @param p_global 输出量：全局坐标系的导航点
   */
  bool TransferL2G(legionclaw::interface::PathPoint p_local,
                   legionclaw::interface::PathPoint& p_global);

  /**
   * @brief 将全局坐标系的导航点坐标,航向角和曲率转换到局部坐标系下。
   * @param point_global 输入量：全局坐标系下起点。
   * @param point_local 输出量：局部坐标系下终点。
   */
  bool TransferG2L(legionclaw::interface::PathPoint point_global,
                   legionclaw::interface::PathPoint& point_local);

  const std::vector<double>& GetOrigin() const;

  bool PathPointNormalizing(const std::vector<double>& in,
                            std::vector<double>& out);

  bool PathPointDeNormalizing(const std::vector<double>& in,
                              std::vector<double>& out);

  bool PathPointNormalizing(const Vec2d& in, Vec2d& out);

  bool PathPointDeNormalizing(const Vec2d& in, Vec2d& out);

private:
  double m_x0_; // 成员变量：局部坐标的坐标原点在全局坐标系下的x值。
  double m_y0_; // 成员变量：局部坐标的坐标原点在全局坐标系下的y值。
  double m_angle_; // 成员变量：局部坐标系y轴指向。
  std::vector<double> translate_origin_;
};
} // namespace math
} // namespace common
} // namespace legionclaw
