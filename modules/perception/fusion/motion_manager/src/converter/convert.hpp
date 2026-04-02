/** 
 * @Date         : 2022-02-20 12:45:44
 * @LastEditors  : lx <m335800283@outlook.com>
 * @LastEditTime : 2022-03-03 19:58:02
 * @FilePath     : /new_1228/legionclaw-framework_1223/modules/perception/fusion/preprocessor/src/converter/convert.hpp
 **/
#ifndef __CONVERT_HPP__
#define __CONVERT_HPP__

#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/radar_obstacle_list_msg.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/time.hpp"
#include "modules/common/interface/polygon_2d.hpp"
#include "modules/common/interface/ultrasonic.hpp"
#include "modules/common/enum/enum.h"
#include "coordinate.hpp"

namespace legionclaw
{
    namespace preprocessor
    {
        class convert
        {
        private:
            /* data */
        public:
            convert(/* args */)
            {
            }

            ~convert()
            {
            }
            // done
            static void merge_offset(legionclaw::interface::Point3D *point, double x_offset = 0.0, double y_offset = 0.0, double z_offset = 0.0, double angle_offset = 0.0)
            {
                double x = point->x() + x_offset;
                double y = point->y() + y_offset;
                point->set_x(x * cos(angle_offset * M_PI / 180.0) - y * sin(angle_offset * M_PI / 180.0));
                point->set_y(y * cos(angle_offset * M_PI / 180.0) + x * sin(angle_offset * M_PI / 180.0));

                double z = point->z() + z_offset;
                point->set_z(z);
            }

            // done
            static void obstacle_append(legionclaw::interface::ObstacleList *to, legionclaw::interface::ObstacleList *from)
            {
                std::vector<legionclaw::interface::Obstacle> from_fused_object_array;
                std::vector<legionclaw::interface::Obstacle> to_fused_object_array;
                from->obstacle(from_fused_object_array);
                to->obstacle(to_fused_object_array);
                size_t obj_size = from_fused_object_array.size();
                for (size_t i = 0; i < obj_size; i++)
                {
                    if (from_fused_object_array[i].fusion_type() != legionclaw::interface::Obstacle::FusionType::FUSION_TYPE_UNKNOWN)
                    {
                        to_fused_object_array.push_back(from_fused_object_array[i]);
                        // std::cout<<"vel_x:"<<from_fused_object_array[i].velocity_abs().x()<<std::endl;
                        // std::cout<<"vel_y:"<<from_fused_object_array[i].velocity_abs().y()<<std::endl;
                        // std::cout<<"absvel_x:"<<from_fused_object_array[i].velocity_abs().x()<<std::endl;
                        // std::cout<<"absvel_y:"<<from_fused_object_array[i].velocity_abs().y()<<std::endl;
                        // std::cout<<"theta:"<<from_fused_object_array[i].theta_abs()<<std::endl;
                    }
                }
                to->clear_obstacle();
                to->set_obstacle(&to_fused_object_array);

                // legionclaw::interface::Header header = from->header();
                // to->set_header(header);
            }
            // done
            //lxprocess_wheel_points
            static void obstacle_append_camera(legionclaw::interface::ObstacleList *to, legionclaw::interface::ObstacleList *from)
            {   
                std::vector<legionclaw::interface::Obstacle> from_fused_object_array;
                std::vector<legionclaw::interface::Obstacle> to_fused_object_array;
                from->obstacle(from_fused_object_array);
                to->obstacle(to_fused_object_array);
                int sensor_id_from = from->sensor_id();
                int sensor_id_to= to->sensor_id();
                // AINFO<<"sensor_id_from: "<<sensor_id_from;
                // AINFO<<"sensor_id_to: "<<sensor_id_to;
                for (auto ob_to:to_fused_object_array)
                {   
                    ob_to.set_timestamp(to->header().stamp());
                }
                for (auto ob_from:from_fused_object_array)
                {
                    if (ob_from.fusion_type() != legionclaw::interface::Obstacle::FusionType::FUSION_TYPE_UNKNOWN)
                    {   
                        //将ob执行append操作后会丢失sensor_id信息
                        // AINFO<<"ob.id(): "<<ob_from.id();
                        // AINFO<<"ob.id()*10+from->sensor_id(): "<<ob_from.id()*10+from->sensor_id();
                        ob_from.set_timestamp(from->header().stamp());
                        ob_from.set_id(ob_from.id()*10+from->sensor_id());
                        // AINFO<<"ob_time: "<<ob_from.timestamp().sec()<<"."<<ob_from.timestamp().nsec();
                        to_fused_object_array.push_back(ob_from);
                    }
                }
                to->clear_obstacle();
                to->set_obstacle(&to_fused_object_array);

            }
            //





            static void convert_point(legionclaw::interface::Point3D *point, legionclaw::interface::Location *location)
            {
                std::vector<double> values = legionclaw::preprocessor::coordinate::convert_point(location->utm_position().x(),
                                                                                             location->utm_position().y(),
                                                                                             location->utm_position().z(),
                                                                                             location->roll(),
                                                                                             location->pitch(),
                                                                                             location->heading(),
                                                                                             point->x(),
                                                                                             point->y(),
                                                                                             point->z());

                point->set_x(values[0]);
                point->set_y(values[1]);
                point->set_z(values[2]);
            }
            static void obstacle_to_world(legionclaw::interface::ObstacleList *data,
                                          legionclaw::interface::Location *location)
            {
                std::vector<legionclaw::interface::Obstacle> data_fused_object_array;
                data->obstacle(data_fused_object_array);
                size_t obj_size = data_fused_object_array.size();

                for (size_t obj_index = 0; obj_index < obj_size; obj_index++) // obstacle copy
                {
                    legionclaw::interface::Point3D to_pos = data_fused_object_array[obj_index].center_pos_vehicle();
                    convert_point(&to_pos, location);
                    data_fused_object_array[obj_index].set_center_pos_abs(to_pos);

                    double theta_vehicle = data_fused_object_array[obj_index].theta_vehicle();
                    double theta_abs = theta_vehicle + location->heading();
                    if (theta_abs > M_PI)
                        theta_abs -= M_PI * 2.0;
                    else if (theta_abs < -M_PI)
                        theta_abs += M_PI * 2.0;
                    

                    data_fused_object_array[obj_index].set_theta_abs(theta_abs);
                    
                    legionclaw::interface::Point3D vel;
                    if(data_fused_object_array[obj_index].fusion_type()==legionclaw::interface::Obstacle::FusionType::LIDAR)
                    {
                        vel = data_fused_object_array[obj_index].velocity_vehicle();
                        legionclaw::preprocessor::coordinate::convert_velocity(&vel, location);
                    }
                    data_fused_object_array[obj_index].set_velocity_abs(vel);

                    
                    std::vector<legionclaw::interface::Point3D> from_polygon;
                    data_fused_object_array[obj_index].polygon_point_vehicle(from_polygon);

                    size_t polygon_point_size = from_polygon.size();
                    std::vector<legionclaw::interface::Point3D> to_polygon;
                    to_polygon.resize(polygon_point_size);
                    for (size_t polygon_point_index = 0; polygon_point_index < polygon_point_size; polygon_point_index++) // polygon_point copy
                    {
                        legionclaw::interface::Point3D pt = from_polygon[polygon_point_index];
                        convert_point(&pt, location);
                        to_polygon[polygon_point_index] = pt;
                    }
                    data_fused_object_array[obj_index].clear_polygon_point_abs();
                    data_fused_object_array[obj_index].set_polygon_point_abs(&to_polygon);

                    legionclaw::interface::Point3D anchor_point = data_fused_object_array[obj_index].anchor_point_vehicle();
                    convert_point(&anchor_point, location);
                    data_fused_object_array[obj_index].set_anchor_point_abs(anchor_point);
                }
                data->clear_obstacle();
                data->set_obstacle(&data_fused_object_array);
            }
        };
    } // namespace preprocessor
} // namespace legionclaw

#endif
