#ifndef QPSPLINEREFERENCELINESMOOTHER_H
#define QPSPLINEREFERENCELINESMOOTHER_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "modules/planning/src/common/reference_line/reference_line.h"
#include "modules/planning/src/common/reference_line/reference_point.h"
#include "modules/planning/src/common/math/include/smoothing_spline/spline_2d_solver.h"
namespace legionclaw
{
namespace reference_line
{
    using legionclaw::planning::math::Spline2dSolver;
    class QpSplineReferenceLineSmoother
    {
    public:
        QpSplineReferenceLineSmoother();
        QpSplineReferenceLineSmoother(const double &anchor_sampling_step, const double &knots_sampling_step, const double &lateral_bound_limit);
        virtual ~QpSplineReferenceLineSmoother() = default;

        bool Smooth(const ReferenceLine &raw_reference_line,
                    ReferenceLine *smoothed_reference_line, const double &path_density, const uint32_t &spline_order);

        void SetAnchorPoints(const std::vector<ReferencePoint> &anchor_points);
    private:
        void Clear();

        //        int GetReferenceLine(  );

        bool Sampling();

        bool AddConstraint();

        bool AddKernel();

        bool Solve();

        std::uint32_t FindIndex(const double t) const;
        int FindIndex(unsigned int j, std::vector<double> t_sampling);


            //        Eigen::MatrixXd constraint_matrix_;
            //
            //        Eigen::MatrixXd constraint_boundary_;

    private:
        std::vector<ReferencePoint> anchor_points_;

        std::vector<double> t_knots_;

        std::vector<double> t_sampling_;

        std::unique_ptr<Spline2dSolver> spline_solver_;

    public:
        uint32_t spline_order_;
        uint32_t total_param_;

        double anchor_sampling_step_ = 0.0;
        double knots_sampling_step_ = 0.0;
        double lateral_bound_limit_ = 0.1;

        double ref_x_ = 0.0;
        double ref_y_ = 0.0;
        double ref_heading_ = 0.0;
        double ref_mileage_ = 0.0;
        double sampling_step_ = 25.0;
    };

} // namespace reference_line
}

#endif // QPSPLINEREFERENCELINESMOOTHER_H
