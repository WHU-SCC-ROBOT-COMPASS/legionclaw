#include <math.h>
#include <cassert>
#include <iostream>
#include "modules/common/math/math_tools.h"
#include "modules/prediction/src/common/spline/cubic_spline.h"
namespace legionclaw
{
    namespace prediction
    {
        namespace spline
        {

            CubicSpline::CubicSpline()
            {
                //ctor
                m_a_ = 0.0;
                m_b_ = 0.0;
                m_c_ = 0.0;
                m_d_ = 0.0;

                m_x_start_ = 0.0;
                m_y_start_ = 0.0;
                m_x_end_ = 0.0;
                m_y_end_ = 0.0;

                m_first_deriv_start_ = 0.0;
                m_first_deriv_end_ = 0.0;
            }

            CubicSpline::~CubicSpline()
            {
                //dtor
            }

            /************************************
设置起止点
************************************/
            int CubicSpline::set_points(double x0, double y0, double x1, double y1)
            {
                m_x_start_ = x0;
                m_y_start_ = y0;
                m_x_end_ = x1;
                m_y_end_ = y1;

                return 1;
            }

            /***************************************************
设置边界条件
deriv[2][3] : { 起点：一阶导，二阶导，三阶导;
                终点：一阶导，二阶导，三阶导 }
***************************************************/
            int CubicSpline::set_boundary(double dy0, double dy1)
            {
                m_first_deriv_start_ = dy0;
                m_first_deriv_end_ = dy1;

                return 1;
            }

            /************************************
计算三次样条曲线系数
************************************/
            int CubicSpline::compute_coef()
            {
                double delta_x = m_x_end_ - m_x_start_;
                double delta_x2 = delta_x * delta_x;
                double delta_x3 = delta_x2 * delta_x;
                double delta_y = m_y_end_ - m_y_start_;

                m_d_ = m_y_start_;
                m_c_ = m_first_deriv_start_;
                m_b_ = (3.0 * delta_y - delta_x * m_first_deriv_end_ - 2.0 * m_c_ * delta_x) / delta_x2;
                m_a_ = (m_c_ * delta_x + delta_x * m_first_deriv_end_ - 2.0 * delta_y) / delta_x3;

                return 1;
            }

            /************************************
拟合后计算任意点的值
x_start < x < x_end
************************************/
            double CubicSpline::operator()(double x) const
            {
                //    assert( x >= m_x_start_ && x <= m_x_end_ );

                double y = 0.0;
                double dx = x - m_x_start_;

                y = m_a_ * pow(dx, 3) + m_b_ * pow(dx, 2) + m_c_ * dx + m_d_;

                return y;
            }

            /************************************
拟合后计算任意点的一阶导
x_start < x < x_end
************************************/
            double CubicSpline::compute_first_deriv(double x)
            {
                //    assert( x >= m_x_start_ && x <= m_x_end_ );

                double first_deriv_y = 0.0;
                double dx = x - m_x_start_;

                first_deriv_y = 3.0 * m_a_ * pow(dx, 2) + 2.0 * m_b_ * dx + m_c_;

                return first_deriv_y;
            }

            /************************************
拟合后计算任意点的二阶导
x_start < x < x_end
************************************/
            double CubicSpline::compute_second_deriv(double x)
            {
                //    assert( x >= m_x_start_ && x <= m_x_end_ );

                double second_deriv_y = 0.0;
                double dx = x - m_x_start_;

                second_deriv_y = 6.0 * m_a_ * dx + 2.0 * m_b_;

                return second_deriv_y;
            }

            /************************************
拟合后计算任意点的加速度
************************************/
            double CubicSpline::compute_accelerate(double x)
            {
                //assert( x >= m_x_start_ && x <= m_x_end_ );

                double acc = 0.0;
                double y = 0.0, dy = 0.0;

                y = operator()(x);
                dy = compute_first_deriv(x);
                acc = y * dy;

                return acc;
            }

            /************************************
提取系数
************************************/
            int CubicSpline::get_coef(double *coef)
            {
                coef[0] = m_a_;
                coef[1] = m_b_;
                coef[2] = m_c_;
                coef[3] = m_d_;

                return 1;
            }

        } // namespace spline
    }
}
