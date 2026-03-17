#ifndef __COORDINATE_HPP__
#define __COORDINATE_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/location.hpp"

namespace legion
{
    namespace preprocessor
    {
        class coordinate
        {
        private:
            /* data */
        public:
            coordinate(/* args */)
            {
            }
            ~coordinate()
            {
            }
            static std::vector<double> get_matrix(double phi, double omega, double kappa)
            {
                std::vector<double> values;
                values.resize(9);
                int index = 0;
                values[index] = cos(phi) * cos(kappa) - sin(phi) * sin(omega) * sin(kappa);
                index++;
                values[index] = -cos(phi) * sin(kappa) - sin(phi) * sin(omega) * cos(kappa);
                index++;
                values[index] = -sin(phi) * cos(omega);
                index++;
                values[index] = cos(omega) * sin(kappa);
                index++;
                values[index] = cos(omega) * cos(kappa);
                index++;
                values[index] = -sin(omega);
                index++;
                values[index] = sin(phi) * cos(kappa) + cos(phi) * sin(omega) * sin(kappa);
                index++;
                values[index] = -sin(phi) * sin(kappa) + cos(phi) * sin(omega) * cos(kappa);
                index++;
                values[index] = cos(phi) * cos(omega);
                return values;
            }
            static std::vector<double> get_rotate(double Geox1, double Geoy1, double Geoz1, double Geox2, double Geoy2, double Geoz2, std::vector<double> mat)
            {
                double t1 = mat[0] * Geox1 +
                            mat[1] * Geoy1 + mat[2] * Geoz1;
                double t2 = mat[3] * Geox1 +
                            mat[4] * Geoy1 + mat[5] * Geoz1;
                double t3 = mat[6] * Geox1 +
                            mat[7] * Geoy1 + mat[8] * Geoz1;
                double x = Geox2 + t1;
                double y = Geoy2 + t2;
                double z = Geoz2 + t3;
                std::vector<double> data;
                data.push_back(x);
                data.push_back(y);
                data.push_back(z);
                return data;
            }
            static std::vector<double> convert_point(double vehicle_x, double vechile_y, double vehicle_z,
                                                     double roll, double pitch, double heading,
                                                     double point_x, double point_y, double point_z)
            {
                double phi = roll;
                double omega = pitch;
                double kappa = heading;

                auto mat = get_matrix(phi, omega, kappa);
                return get_rotate(point_x, point_y, point_z,
                                  vehicle_x, vechile_y, vehicle_z,
                                  mat);
            }

            static void convert_velocity(legion::interface::Point3D *vel_vehicle,
                                         legion::interface::Location *location)
            {
                //20231123 针对值为128的相对速度，绝对速度赋0
                if(vel_vehicle->x() == 128 && vel_vehicle->y() == 128)
                {
                    vel_vehicle->set_x(0);
                    vel_vehicle->set_y(0);
                }else
                {
                    double vehicle_velocity = std::hypot(location->linear_velocity().x(), location->linear_velocity().y());
                     // cout << "vehicle_velocity: " << vel_vehicle->x() << endl;
                    double vel_x = vel_vehicle->x() + vehicle_velocity;
                    double vel_y = vel_vehicle->y();

                    double yaw = location->heading();
                    
                    double x1 = vel_x * cos(yaw);
                    double x2 = -vel_y * sin(yaw);
                    double y1 = vel_y * cos(yaw);
                    double y2 = vel_x * sin(yaw);
                    // cout<<"vx: "<<vel_x<<endl;
                    // cout<<"vy: "<<vel_y<<endl;
                    vel_vehicle->set_x(x1 + x2);
                    vel_vehicle->set_y(y1 + y2);
                }     
            }
            static void convert_acc(legion::interface::Point3D *vel_vehicle,
                                         legion::interface::Location *location)
            {
                double vehicle_acc = std::hypot(location->linear_acceleration().x(), location->linear_acceleration().y());
                double vel_x = vel_vehicle->x() + vehicle_acc;
                double vel_y = vel_vehicle->y();

                double yaw = location->heading();
                
                double x1 = vel_x * cos(yaw);
                double x2 = -vel_y * sin(yaw);
                double y1 = vel_y * cos(yaw);
                double y2 = vel_x * sin(yaw);

                vel_vehicle->set_x(x1 + x2);
                vel_vehicle->set_y(y1 + y2);
            }

            static void to_vehicle_position(double from_x, double from_y, double from_z,
                                            double &to_x, double &to_y, double &to_z,
                                            double vehicle_x, double vehicle_y, double vehicle_z,
                                            double roll, double pitch, double yaw)
            {
                double roll_rad = roll * M_PI / 180.0;
                double pitch_rad = pitch * M_PI / 180.0;
                double yaw_rad = yaw * M_PI / 180.0;

                Eigen::Vector3d eulerAngle(yaw_rad, pitch_rad, roll_rad);
                Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
                Eigen::AngleAxisd rotation_vector;
                rotation_vector = yawAngle * pitchAngle * rollAngle;

                Eigen::Vector3d v(from_x, from_y, from_z);
                Eigen::Vector3d v_rotated = rotation_vector * v;
                to_x = v_rotated.x() + vehicle_x;
                to_y = v_rotated.y() + vehicle_y;
                to_z = v_rotated.z() + vehicle_z;
            }
            static void to_vehicle_velocity(double from_vel_x, double from_vel_y, double from_vel_z,
                                            double &to_vel_x, double &to_vel_y, double &to_vel_z,
                                            double vehicle_x, double vehicle_y, double vehicle_z,
                                            double roll, double pitch, double yaw)
            {
                double roll_rad = roll * M_PI / 180.0;
                double pitch_rad = pitch * M_PI / 180.0;
                double yaw_rad = yaw * M_PI / 180.0;

                Eigen::Vector3d eulerAngle(yaw_rad, pitch_rad, roll_rad);
                Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
                Eigen::AngleAxisd rotation_vector;
                rotation_vector = yawAngle * pitchAngle * rollAngle;

                Eigen::Vector3d v(from_vel_x, from_vel_y, from_vel_z);
                Eigen::Vector3d v_rotated = rotation_vector * v;
                to_vel_x = v_rotated.x();
                to_vel_y = v_rotated.y();
                to_vel_z = v_rotated.z();
            }
        };

    } // namespace preprocessor
} // namespace legion
#endif