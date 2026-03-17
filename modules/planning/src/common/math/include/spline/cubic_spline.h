/**
 * @file
 *
 * @brief 三次样条拟合库，
 */

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>

using namespace std;
namespace legionclaw {
namespace planning {
namespace spline {

/**
 * @class CubicSpline
 * @brief 根据两点的状态，通过四次样条拟合出一条平滑的曲线，
 * 并可以内插出曲线内任意一点的坐标，计算该点的方向角和曲率。
 */
class CubicSpline {
 public:
  /**
   * @brief 构造函数
   */
  CubicSpline();
  /**
   * @brief 析构函数
   */
  virtual ~CubicSpline();

  /**
   * @brief 设置起点和终点坐标,
   * @param x0 输入量：起点x坐标。
   * @param y0 输入量：起点y坐标。
   * @param x1 输入量：终点x坐标。
   * @param y1 输入量：终点y坐标。
   */
  int set_points(double x0, double y0, double x1, double y1);

  /**
   * @brief 设置三次样条函数的边界条件。
   * @param dy0 输入量：起点的一阶导。
   * @param dy1 输入量：终点的一阶导。。
   */
  int set_boundary(double dy0, double dy1);

  /**
   * @brief 求解三次样条函数的系数
   */
  int compute_coef();

  /**
   * @brief 给定曲线范围内任一x值，根据三次样条函数求解对应的y值,
   * @param x 输入量：曲线范围内任一x值。
   * @return x对应的y值。
   */
  double operator()(double x) const;

  /**
   * @brief 给定曲线范围内任一x值，根据三次样条函数求解y的一阶导,
   * @param x 输入量：曲线范围内任一x值。
   * @return y的一阶导。
   */
  double compute_first_deriv(double x);

  /**
   * @brief 给定曲线范围内任一x值，根据三次样条函数求解y的二阶导,
   * @param x 输入量：曲线范围内任一x值。
   * @return y的二阶导。
   */
  double compute_second_deriv(double x);

  /**
   * @brief 给定曲线范围内任一x值，根据三次样条函数求解该点的加速度,
   * @param x 输入量：曲线范围内任一x值。
   * @return 该点加速度。
   */
  double compute_accelerate(double x);

  /**
   * @brief 获得三次样条函数的系数,
   * @param coef 输出量：三次样条函数的系数。
   */
  int get_coef(double *coef);

 protected:
 private:
  ///成员变量：四次样条函数的系数。
  double m_a_, m_b_, m_c_, m_d_;
  ///成员变量：起点和终点的坐标。
  double m_x_start_, m_y_start_, m_x_end_, m_y_end_;
  ///成员变量：起点和终点的坐标的一阶导。
  double m_first_deriv_start_, m_first_deriv_end_;
};

}  // namespace spline
}  // namespace planning
}  // namespace legionclaw

#endif  // CUBIC_SPLINE_H
