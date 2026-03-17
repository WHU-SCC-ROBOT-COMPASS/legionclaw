#include <lanelet2_ex.h>

namespace lanelet2_ex
{

    //返回正常以x轴正方向为0，逆时针增加的角度（-180~180）
    double calculate_angle(lanelet::ConstPoint3d front, lanelet::ConstPoint3d behind)
    {
        double dx = behind.x() - front.x();
        double dy = behind.y() - front.y();
        double yaw = atan2(dy, dx) * 180 / M_PI;
        return yaw;
    }
    //计算两个角度制的夹角（返回0~180）
    double calculate_diff_angle(double first, double second)
    {
        double diff = first - second;

        if (diff < 0)
            diff = second - first;
        while (diff>360.0)
        {
           diff-=360.0;
        }
        
        if (diff > 180.0)
            diff = 360.0 - diff;

        return diff;
    }

    lanelet::Ids GetClosestLanelets(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point)
    {
        lanelet::Ids ids_;
        for (auto lane : map->laneletLayer)
        {
            if (boost::geometry::covered_by(point.basicPoint2d(), lane.polygon2d()))
            {
                ids_.push_back(lane.id());
            }
        }
        return ids_;
    }

    lanelet::Ids GetClosestLaneletsInRange(const lanelet::LaneletMapConstPtr &map, const lanelet::Point3d point, const double range)
    {
        lanelet::Ids ids_;
        for (auto lane : map->laneletLayer)
        {
            if (lanelet::geometry::distance2d(point, lane.polygon3d()) < range)
            {
                ids_.push_back(lane.id());
            }
        }
        return ids_;
    }

    int GetClosestPoints_Lanelets(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, lanelet::ConstPoints3d &cpoints, lanelet::Ids &ids)
    {
        cpoints.clear();
        ids.clear();
        lanelet::Ids lane_ids = GetClosestLanelets(map, point);
        double d = 0;
        double min_d = DBL_MAX;
        for (auto id : lane_ids)
        {
            lanelet::Lanelet lane = map->laneletLayer.get(id);
            lanelet::ConstLineString3d cline = lane.centerline();
            lanelet::ConstPoint3d front, behind;
            bool belast = false;
            if (cline.size() < 2)
                continue;
            for (unsigned int i = 0; i < cline.size(); i++)
            {
                d = lanelet::geometry::distance2d(cline[i], point);
                if (d < min_d)
                {
                    min_d = d;
                    if (i < cline.size() - 1)
                    {
                        front = cline[i];
                        behind = cline[i + 1];
                    }
                    else
                    {
                        front = cline[i - 1];
                        behind = cline[i];
                        belast = true;
                    }
                }
            }
            if (!bDirectionBased)
            {
                if (!belast)
                {
                    cpoints.push_back(front);
                    ids.push_back(lane.id());
                }
                else
                {
                    cpoints.push_back(behind);
                    ids.push_back(lane.id());
                }
            }
            else
            {
                double yaw = calculate_angle(front, behind);
                double angle = 90.0 - theta;                             //航向角转正常角度
                if (calculate_diff_angle(yaw, angle) <= _MIN_DIFF_ANGLE) //夹角小于60度
                {
                    if (!belast)
                    {
                        cpoints.push_back(front);
                        ids.push_back(lane.id());
                    }
                    else
                    {
                        cpoints.push_back(behind);
                        ids.push_back(lane.id());
                    }
                }
            }
        }
        return cpoints.size();
    }

    int GetClosestPoints_LaneletsInRange(const lanelet::LaneletMapConstPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, lanelet::ConstPoints3d &cpoints, lanelet::Ids &ids, const double range)
    {
        cpoints.clear();
        ids.clear();
        lanelet::Ids lane_ids = GetClosestLaneletsInRange(map, point, range);
        double d = 0;
        double min_d = DBL_MAX;
        for (auto id : lane_ids)
        {
            lanelet::ConstLanelet lane = map->laneletLayer.get(id);
            lanelet::ConstLineString3d cline = lane.centerline();
            lanelet::ConstPoint3d front, behind;
            bool belast = false;
            if (cline.size() < 2)
                continue;
            for (unsigned int i = 0; i < cline.size(); i++)
            {
                d = lanelet::geometry::distance2d(cline[i], point);
                if (d < min_d)
                {
                    min_d = d;
                    if (i < cline.size() - 1)
                    {
                        front = cline[i];
                        behind = cline[i + 1];
                    }
                    else
                    {
                        front = cline[i - 1];
                        behind = cline[i];
                        belast = true;
                    }
                }
            }
            if (!bDirectionBased)
            {
                if (!belast)
                {
                    cpoints.push_back(front);
                    ids.push_back(lane.id());
                }
                else
                {
                    cpoints.push_back(behind);
                    ids.push_back(lane.id());
                }
            }
            else
            {
                double yaw = calculate_angle(front, behind);
                double angle = 90.0 - theta;                             //航向角转正常角度
                if (calculate_diff_angle(yaw, angle) <= _MIN_DIFF_ANGLE) //夹角小于60度
                {
                    if (!belast)
                    {
                        cpoints.push_back(front);
                        ids.push_back(lane.id());
                    }
                    else
                    {
                        cpoints.push_back(behind);
                        ids.push_back(lane.id());
                    }
                }
            }
        }
        return cpoints.size();
    }

    std::pair<lanelet::ConstPoint3d, lanelet::Id> GetClosestPoint_LaneletInRange(const lanelet::LaneletMapConstPtr &map, const lanelet::Point3d point, const double theta, const double range, const double weight_times,const double height_range)
    {
        std::pair<lanelet::ConstPoint3d, lanelet::Id> closest_pair;
        lanelet::Ids lane_ids = GetClosestLaneletsInRange(map, point, range);
        //拿2d去搜索范围内的结果，然后进行3d计算距离，找到最近的，理论上不存在问题
        double min_total = DBL_MAX;
        double min_hang_diff=DBL_MAX;
        double temp_d_total = 0;
        closest_pair.second=-1;
        for (auto id : lane_ids)
        {
            double d = 0;
            double min_d = DBL_MAX;
            lanelet::ConstLanelet lane = map->laneletLayer.get(id);
            lanelet::ConstLineString3d cline = lane.centerline();
            lanelet::ConstPoint3d front, behind;
            bool belast = false;
            if (cline.size() < 2)
                continue;
            for (unsigned int i = 0; i < cline.size(); i++)
            {
                d = lanelet::geometry::distance3d(cline[i], point);
                if (d < min_d)
                {
                    min_d = d;
                    if (i < cline.size() - 1)
                    {
                        front = cline[i];
                        behind = cline[i + 1];
                    }
                    else
                    {
                        front = cline[i - 1];
                        behind = cline[i];
                        belast = true;
                    }
                }
            }
            double true_min_d = lanelet::geometry::distance2d(cline, point);
            double yaw = calculate_angle(front, behind);
            double angle = 180*theta/M_PI; //航向角转正常角度
            double hang_diff = calculate_diff_angle(yaw, angle);
            temp_d_total = hang_diff * weight_times / 180.0 + true_min_d;
            double dis_z=fabs(point.z()-front.z());
            // if(temp_d_total > range || dis_z>height_range)
            // {
            //     continue;
            // }
            if (temp_d_total < min_total)
            {
                min_total = temp_d_total;
                lanelet::ConstPoint3d temp_point;
                temp_point = belast ? behind : front;
                closest_pair.first = temp_point;
                closest_pair.second = id;
                min_hang_diff=hang_diff;
            }
        }
        double dis_z=fabs(point.z()-closest_pair.first.z());
        if(min_total>range|| dis_z>height_range)
        {
            std::cout<<"min dis :"<<min_total<<"\n";
            std::cout<<"heading diff :"<<min_hang_diff<<"\n";
            std::cout<<"locate warning  closest point :"<<closest_pair.first<<"\n";
            std::cout<<"location : "<<" x : "<<point.x()<<" y : "<<point.y()<<" z : "<<point.z()<<"\n";
            closest_pair.second=-1;
        }
        return closest_pair;
    }

    std::pair<lanelet::ConstPoint3d, lanelet::Id> GetClosestPoint_LaneletInRange(const lanelet::Lanelets &lanes, const lanelet::Point3d point, const double theta, const double range, const double weight_times,const double height_range)
    {
        std::pair<lanelet::ConstPoint3d, lanelet::Id> closest_pair;
        double min_total = DBL_MAX;
        double temp_d_total = 0;
        closest_pair.second=-1;
        double min_hang_diff=DBL_MAX;
        for (auto lane_in : lanes)
        {
            double d = 0;
            double min_d = DBL_MAX;
            lanelet::ConstLineString3d cline = lane_in.centerline();
            lanelet::ConstPoint3d front, behind;
            bool belast = false;
            if (cline.size() < 2)
                continue;
            for (unsigned int i = 0; i < cline.size(); i++)
            {
                d = lanelet::geometry::distance3d(cline[i], point);
                if (d < min_d)
                {
                    min_d = d;
                    if (i < cline.size() - 1)
                    {
                        front = cline[i];
                        behind = cline[i + 1];
                    }
                    else
                    {
                        front = cline[i - 1];
                        behind = cline[i];
                        belast = true;
                    }
                }
            }
            double true_min_d = lanelet::geometry::distance2d(cline, point);
            double yaw = calculate_angle(front, behind);
            double angle = 180.0*theta/M_PI; //弧度转角度
            double hang_diff = calculate_diff_angle(yaw, angle);
            temp_d_total = hang_diff * weight_times / 180.0 + true_min_d;
            double dis_z=fabs(point.z()-front.z());
            // if(temp_d_total > range || dis_z>height_range)
            // {
            //     continue;
            // }
            if (temp_d_total < min_total)
            {
                min_total = temp_d_total;
                lanelet::ConstPoint3d temp_point;
                temp_point = belast ? behind : front;
                closest_pair.first = temp_point;
                closest_pair.second = lane_in.id();
                min_hang_diff=hang_diff;
            }
        }
        double dis_z=fabs(point.z()-closest_pair.first.z());
        if(min_total>range|| dis_z>height_range)
        {
            std::cout<<"min dis :"<<min_total<<"\n";
            std::cout<<"heading diff :"<<min_hang_diff<<"\n";
            std::cout<<"locate warning  closest point :"<<closest_pair.first<<"\n";
            std::cout<<"location : "<<" x : "<<point.x()<<" y : "<<point.y()<<" z : "<<point.z()<<"\n";
            closest_pair.second=-1;
        }
        return closest_pair;
    }

    std::pair<lanelet::ConstPoint3d, lanelet::Id> LocatePointOnCenterLine(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased)
    {
        lanelet::ConstPoints3d cpoints;
        lanelet::Ids lane_ids;
        double d = 0;
        double min_d = DBL_MAX;
        int close_index;
        GetClosestPoints_Lanelets(map, point, theta, bDirectionBased, cpoints, lane_ids);
        if (cpoints.size())
        {
            for (unsigned int i = 0; i < cpoints.size(); i++)
            {
                d = lanelet::geometry::distance2d(cpoints[i], point);
                if (d < min_d)
                {
                    close_index = i;
                    min_d=d;
                }
            }
            return std::make_pair(cpoints[close_index], lane_ids[close_index]);
        }
    }

    std::pair<lanelet::ConstPoint3d, lanelet::Id> LocatePointOnCenterLineInRange(const lanelet::LaneletMapConstPtr &map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, const double range)
    {
        lanelet::ConstPoints3d cpoints;
        lanelet::Ids lane_ids;
        double d = 0;
        double min_d = DBL_MAX;
        int close_index;
        GetClosestPoints_LaneletsInRange(map, point, theta, bDirectionBased, cpoints, lane_ids, range);
        if (cpoints.size())
        {
            for (unsigned int i = 0; i < cpoints.size(); i++)
            {
                d = lanelet::geometry::distance2d(cpoints[i], point);
                if (d < min_d)
                {
                    close_index = i;
                    min_d=d;
                }
            }
            return std::make_pair(cpoints[close_index], lane_ids[close_index]);
        }
    }

    std::pair<double, double> GetLaneBoundDistance(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta)
    {
        std::pair<lanelet::ConstPoint3d, lanelet::Id> close_ = LocatePointOnCenterLine(map, point, theta, true);
        lanelet::Lanelet lane_ = map->laneletLayer.get(close_.second);
        double left_d = lanelet::geometry::distance2d(lane_.leftBound2d().basicLineString(), point.basicPoint2d());
        double right_d = lanelet::geometry::distance2d(lane_.rightBound2d().basicLineString(), point.basicPoint2d());
        return std::make_pair(left_d, right_d);
    }

    lanelet::Segment3d FIndCLoseSegment(const lanelet::Point3d point, lanelet::LineString3d line)
    {
        double min_distance = DBL_MAX;
        lanelet::Segment3d close_seg;
        for (unsigned int i = 0; i < line.numSegments(); i++)
        {
            lanelet::Segment3d temp_seg = line.segment(i);
            lanelet::LineString3d temp_line(lanelet::utils::getId(), {temp_seg.first, temp_seg.second});
            double d = lanelet::geometry::distance2d(temp_line.basicLineString(), point.basicPoint2d());
            if (d < min_distance)
            {
                close_seg = temp_seg;
            }
        }
        return close_seg;
    }

    lanelet::Point3d FixLocationPoint(const lanelet::LaneletMapUPtr &map, const lanelet::Point3d point, const double theta, std::pair<double, double> distances, lanelet2_ex::Trajectory_dir fixdir)
    {
        std::pair<lanelet::ConstPoint3d, lanelet::Id> close_ = LocatePointOnCenterLine(map, point, theta, true);
        lanelet::Lanelet lane_ = map->laneletLayer.get(close_.second);
        if (fixdir == lanelet2_ex::Trajectory_dir::LEFT)
        {
            lanelet::Segment3d close_seg = FIndCLoseSegment(point, lane_.leftBound());
            lanelet::Point3d p1 = close_seg.first;
            lanelet::Point3d p2 = close_seg.second;
            auto v1 = std::make_pair(p2.x() - p1.x(), p2.y() - p1.y());                                                                                                         //向量1
            auto v2 = std::make_pair(point.x() - p1.x(), point.y() - p1.y());                                                                                                   //向量2
            double length2 = pow(v1.first, 2) + pow(v1.second, 2);                                                                                                              //向量1长度的平方
            auto v_l = std::make_pair(v1.first * (v1.first * v2.first + v1.second * v2.second) / length2, v1.second * (v1.first * v2.first + v1.second * v2.second) / length2); //向量2在1上的投影
            // double v1v2cos=(v1.first*v2.first+v1.second*v2.second)/(sqrt(pow(v1.first,2)+pow(v1.second,2))*sqrt(pow(v2.first,2)+pow(v2.second,2)));//向量1和2的夹角cos值
            // auto v_l=std::make_pair(v2.first*v1v2cos,v2.second*v1v2cos);//向量2在1上的投影
            auto point_l = std::make_pair(v_l.first + p1.x(), v_l.second + p1.y());          //垂点坐标
            auto v3 = std::make_pair(point.x() - point_l.first, point.y() - point_l.second); //垂直向量
            auto v3_length = sqrt(pow(v3.first, 2) + pow(v3.second, 2));
            if (v3_length < 0.0001)
            {
                return point;
            }
            auto size = distances.first / v3_length;                                                               //垂直向量和给定距离的比例
            auto final_point = std::make_pair(v3.first * size + point_l.first, v3.second * size + point_l.second); // 偏移点坐标
            lanelet::Point3d fix_point(lanelet::utils::getId(), final_point.first, final_point.second, 0);
            return fix_point;
        }
        else if (fixdir == lanelet2_ex::Trajectory_dir::RIGHT)
        {
            lanelet::Segment3d close_seg = FIndCLoseSegment(point, lane_.rightBound());
            lanelet::Point3d p1 = close_seg.first;
            lanelet::Point3d p2 = close_seg.second;
            auto v1 = std::make_pair(p2.x() - p1.x(), p2.y() - p1.y());                                                                                                         //向量1
            auto v2 = std::make_pair(point.x() - p1.x(), point.y() - p1.y());                                                                                                   //向量2
            double length2 = pow(v1.first, 2) + pow(v1.second, 2);                                                                                                              //向量1长度的平方
            auto v_l = std::make_pair(v1.first * (v1.first * v2.first + v1.second * v2.second) / length2, v1.second * (v1.first * v2.first + v1.second * v2.second) / length2); //向量2在1上的投影
            auto point_l = std::make_pair(v_l.first + p1.x(), v_l.second + p1.y());                                                                                             //垂点坐标
            auto v3 = std::make_pair(point.x() - point_l.first, point.y() - point_l.second);                                                                                    //垂直向量
            auto v3_length = sqrt(pow(v3.first, 2) + pow(v3.second, 2));
            if (v3_length < 0.0001)
            {
                return point;
            }
            auto size = distances.first / v3_length;                                                               //垂直向量和给定距离的比例
            auto final_point = std::make_pair(v3.first * size + point_l.first, v3.second * size + point_l.second); // 偏移点坐标
            lanelet::Point3d fix_point(lanelet::utils::getId(), final_point.first, final_point.second, 0);
            return fix_point;
        }
        else
        {
            return point;
        }
    }

    //  找到中心线上点的索引（或者距离该点最近的中心线上的点的索引）
    unsigned int GetPointIndex(const lanelet::ConstPoint3d point, const lanelet::ConstLineString3d cline)
    {
        double min_d = DBL_MAX;
        double temp_d = 0;
        unsigned int index = 0;
        for (unsigned int i = 0; i < cline.size(); i++)
        {
            temp_d = lanelet::geometry::distance2d(point.basicPoint2d(), cline[i].basicPoint2d());
            if (temp_d < min_d)
            {
                index = i;
                min_d=temp_d;
            }
        }
        return index;
    }

    // 将path加入到paths的开头
    int LinkPath(lanelet::ConstPoints3d path, std::vector<lanelet::ConstPoints3d> &paths)
    {
        for (auto item : paths)
        {
            item.insert(item.begin(), path.begin(), path.end());
        }
        return paths.size();
    }

    //从一个车道开始找寻轨迹（不考虑换道，只考虑后继）
    // std::vector<lanelet::ConstPoints3d> GetPaths(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map ,const lanelet::Id lane_id, const double max_distance, const int index=0)
    // {
    //     lanelet::Lanelet cur_lane=map->laneletLayer.get(lane_id);
    //     lanelet::ConstLineString3d cur_cline=cur_lane.centerline();
    //     double distance=0;
    //     lanelet::ConstPoints3d cur_path;
    //     std::vector<lanelet::ConstPoints3d> cur_paths;
    //     cur_path.clear();
    //     cur_paths.clear();
    //     if(index<cur_cline.size())
    //     {
    //         for(int i=index;i<cur_cline.size()-1;i++)
    //         {
    //             distance+=lanelet::geometry::distance2d(cur_cline[i].basicPoint2d(),cur_cline[i+1].basicPoint2d());
    //             if(distance<max_distance)
    //             {
    //                 cur_path.push_back(cur_cline[i+1]);
    //             }
    //             else break;
    //         }
    //         if(distance>max_distance)
    //         {
    //             cur_paths.push_back(cur_path);
    //             return cur_paths;
    //         }
    //         else
    //         {
    //             lanelet::ConstLanelets follows = routing_map->following(cur_lane);
    //             if(follows.empty())
    //             {
    //                 cur_paths.push_back(cur_path);
    //                 return cur_paths;
    //             }
    //             else
    //             {
    //                 std::vector<lanelet::ConstPoints3d> all_paths;
    //                 all_paths.clear();
    //                 for(auto follow:follows)
    //                 {
    //                     std::vector<lanelet::ConstPoints3d> follow_paths;
    //                     follow_paths.clear();
    //                     lanelet::ConstPoint3d follow_point= follow.centerline()[1];
    //                     if(lanelet::geometry::distance2d(cur_path.back().basicPoint2d(),follow_point.basicPoint2d())+distance>max_distance)
    //                     {
    //                         follow_paths.push_back(cur_path);
    //                     }
    //                     else
    //                     {
    //                         follow_paths= lanelet2_ex::GetPaths(map,routing_map,follow.id(),max_distance-distance,0);
    //                         LinkPath(cur_path,follow_paths);
    //                     }
    //                     all_paths.insert(all_paths.end(),follow_paths.begin(),follow_paths.end());
    //                 }
    //                 return all_paths;
    //             }
    //         }
    //     }
    // }

    /*
    std::vector<std::pair<lanelet::Points3d,lanelet2_ex::Trajectory_dir>> GetTrajectorys(const lanelet::LaneletMapUPtr map, const lanelet::routing::RoutingGraphUPtr routing_map,  const lanelet::Point3d point, const double theta, const bool bDirectionBased, const std::set<lanelet::Ids> besides_lanelet, const double max_distance)
    {
        lanelet::ConstPoints3d cpoints;
        lanelet::Ids lane_ids;
        GetClosestPoints_Lanelets(map,level,point,theta,bDirectionBased,cpoints,lane_ids);
        std::vector<lanelet::Points3d> all_paths;//所有可能轨迹中心线
        for(int i=0;i<cpoints.size();i++)
        {
            lanelet::ConstPoint3d point_=cpoints[i];
            lanelet::Lanelet lane_=map->laneletLayer.get(lane_ids[i]);
            lanelet::Ids slice;
            int index=0;//车道位置索引
            bool find_slice=false;
            for(auto set:besides_lanelet)
            {
                for(int j=0;j<set.size();j++)
                {
                    if(set[j]==lane_.id())
                    {
                        slice=set;
                        index=j;
                        find_slice=true;
                        break;
                    }
                }
                if(find_slice)
                break;
            }
            bool toleft=false,toright=false;
            auto left_type=lane_.leftBound().attributeOr(lanelet::AttributeName::Subtype,"unknown");
            auto right_type=lane_.rightBound().attributeOr(lanelet::AttributeName::Subtype,"unknown");
            if(left_type==lanelet::AttributeValueString::Dashed||left_type==lanelet::AttributeValueString::SolidDashed)
            {
                if(index>0)
                toleft=true;
            }
            if(right_type==lanelet::AttributeValueString::Dashed||right_type==lanelet::AttributeValueString::DashedSolid)
            {
                if(index<slice.size()-1)
                toright=true;
            }
            auto self_paths=GetPaths(map,routing_map,lane_.id(),max_distance,GetPointIndex(point_,lane_.centerline()));
            all_paths.insert(all_paths.end(),self_paths.begin(),self_paths.end());
            if(toleft)
            {
                auto left=map->laneletLayer.get(slice[index-1]);
                auto left_paths=GetPaths(map,routing_map,left.id(),max_distance,GetPointIndex(point_,left.centerline()));
                all_paths.insert(all_paths.end(),left_paths.begin(),left_paths.end());
            }
            if(toright)
            {
                auto right=map->laneletLayer.get(slice[index+1]);
                auto right_paths=GetPaths(map,routing_map,right.id(),max_distance,GetPointIndex(point_,right.centerline()));
                all_paths.insert(all_paths.end(),right_paths.begin(),right_paths.end());
            }
            
        }
    }

    */

    //将道路连接起来
    std::vector<lanelet::Ids> LinkLanes(const lanelet::Id start_id, std::vector<lanelet::Ids> paths)
    {
        for (auto path : paths)
        {
            path.insert(path.begin(), start_id);
        }
        return paths;
    }

    //返回包括start_id对应的lanelet在内的paths
    std::vector<lanelet::Ids> GetFollowsInDistance(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, lanelet::Id start_id, double max_distance)
    {
        std::vector<lanelet::Ids> paths;
        lanelet::Ids ids_;
        paths.clear();
        ids_.clear();
        lanelet::Lanelet lane_ = map->laneletLayer.get(start_id);
        double self_length = lanelet::geometry::length2d(lane_);
        auto follows = routing_map->following(lane_);
        if (self_length >= max_distance || follows.empty())
        {
            ids_.push_back(start_id);
            paths.push_back(ids_);
        }
        else
        {
            for (auto follow : follows)
            {
                auto follow_paths = GetFollowsInDistance(map, routing_map, follow.id(), max_distance - self_length);
                paths = LinkLanes(start_id, follow_paths);
            }
        }
        return paths;
    }

    //+根据当前车道id找到是否有左右车道，和左右车道id
    std::pair<bool, bool> GetSlice(lanelet::Id lane_id, const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, std::pair<lanelet::Id, lanelet::Id> &besides)
    {
        auto lane_ = map->laneletLayer.get(lane_id);
        lanelet::ConstLanelets slice = routing_map->besides(lane_);
        unsigned int index = 0;
        for (unsigned int i = 0; i < slice.size(); i++)
        {
            if (slice[i].id() == lane_id)
            {
                index = i;
                break;
            }
        }
        bool toleft = false, toright = false;
        if (index > 0)
        {
            besides.first = slice[index - 1].id();
            toleft = true;
        }
        if (index < slice.size() - 1)
        {
            besides.second = slice[index + 1].id();
            toright = true;
        }

        return std::make_pair(toleft, toright);
    }

    //获取最近中心点索引和剩余道路距离
    std::pair<double, int> GetRemainDistance(const lanelet::ConstPoint3d point, const lanelet::ConstLineString3d cline)
    {
        unsigned int index = GetPointIndex(point, cline);
        if (index == cline.size())
        {
            return std::make_pair(0, cline.size());
        }
        double distance = 0;
        for (unsigned int i = index; i < cline.size() - 1; i++)
        {
            distance += lanelet::geometry::distance3d(cline[i], cline[i + 1]);
        }
        return std::make_pair(distance, index);
    }

    //paths从ids形式，转为points形式
    std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> Ids2Lines(const lanelet::LaneletMapUPtr &map, lanelet2_ex::Trajectory_dir dir, std::vector<lanelet::Ids> paths)
    {
        std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> result;
        for (auto path : paths)
        {
            lanelet::ConstPoints3d line;
            line.clear();
            for (auto lane_id : path)
            {
                lanelet::Lanelet lane = map->laneletLayer.get(lane_id);
                lanelet::ConstLineString3d cline = lane.centerline();
                line.insert(line.end(), cline.begin(), cline.end() - 1);
            }
            result.push_back(std::make_pair(line, dir));
        }
        return result;
    }

    std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Lan_Atr>> Ids2LinesWithTag(const lanelet::LaneletMapUPtr &map, lanelet2_ex::Lan_Atr attr, std::vector<lanelet::Ids> paths)
    {
        std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Lan_Atr>> result;
        for (auto path : paths)
        {
            lanelet::ConstPoints3d line;
            line.clear();
            for (auto lane_id : path)
            {
                lanelet::Lanelet lane = map->laneletLayer.get(lane_id);
                lanelet::ConstLineString3d cline = lane.centerline();
                line.insert(line.end(), cline.begin(), cline.end() - 1);
            }
            result.push_back(std::make_pair(line, attr));
        }
        return result;
    }

    //将一系列lanelets 中有前驱后继关系的链接在一起
    std::vector<lanelet::Ids> LinkFollows(const lanelet::LaneletMapUPtr &map, lanelet::Ids ids)
    {
        std::vector<lanelet::Ids> paths;
        paths.clear();
        if (ids.size() <= 1)
        {
            paths.push_back(ids);
            return paths;
        }
        lanelet::Ids start_ids;
        start_ids.clear();
        std::map<lanelet::Id, lanelet::Id> follow_map;
        follow_map.clear();
        for (auto id : ids)
        {
            lanelet::Lanelet self = map->laneletLayer.get(id);
            bool hasprev = false;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                auto temp = map->laneletLayer.get(ids[j]);
                bool thisprev = lanelet::geometry::follows(temp, self);
                hasprev = hasprev & thisprev;
                if (thisprev)
                {
                    follow_map.insert(std::make_pair(ids[j], id));
                }
            }
            if (!hasprev)
            {
                start_ids.push_back(id);
            }
        }
        if (start_ids.empty())
            start_ids.push_back(ids[0]);
        for (auto start_id : start_ids)
        {
            lanelet::Ids path;
            path.clear();
            path.push_back(start_id);
            while (follow_map.find(path.back()) != follow_map.end())
                ;
            {
                path.push_back(follow_map.at(path.back()));
            }
            paths.push_back(path);
        }
        return paths;
    }

    lanelet::ConstPoints3d Cline2Points(lanelet::ConstLineString3d cline)
    {
        lanelet::ConstPoints3d points;
        points.clear();
        for (auto point : cline)
        {
            points.insert(points.end(), point);
        }
    }

    std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> GetRoute(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, const lanelet::Point3d point, const double theta, const bool bDirectionBased, const double max_distance)
    {
        std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Trajectory_dir>> all_trajectory;
        all_trajectory.clear();
        std::pair<lanelet::Id, lanelet::Id> besides;
        auto closest = LocatePointOnCenterLine(map, point, theta, bDirectionBased);
        std::pair<bool, bool> besides_t = GetSlice(closest.second, map, routing_map, besides);
        lanelet::Lanelet lane_ = map->laneletLayer.get(closest.second);
        auto follows = routing_map->following(lane_);
        auto self_remain = GetRemainDistance(point, lane_.centerline()); //first 剩余距离
        if (self_remain.first > max_distance)
        {
            auto self_tra = std::make_pair(Cline2Points(lane_.centerline()), lanelet2_ex::Trajectory_dir::STRAIGHT);
            all_trajectory.push_back(self_tra);
            if (besides_t.first)
            {
                lanelet::Lanelet left = map->laneletLayer.get(besides.first);
                auto left_tra = std::make_pair(Cline2Points(left.centerline()), lanelet2_ex::Trajectory_dir::LEFT);
                all_trajectory.push_back(left_tra);
            }
            if (besides_t.second)
            {
                lanelet::Lanelet right = map->laneletLayer.get(besides.second);
                auto right_tra = std::make_pair(Cline2Points(right.centerline()), lanelet2_ex::Trajectory_dir::RIGHT);
                all_trajectory.push_back(right_tra);
            }
            return all_trajectory;
        }
        else if (self_remain.first < max_distance && follows.empty())
        {
            auto self_tra = std::make_pair(Cline2Points(lane_.centerline()), lanelet2_ex::Trajectory_dir::STRAIGHT);
            all_trajectory.push_back(self_tra);
            if (besides_t.first)
            {
                lanelet::Lanelet left = map->laneletLayer.get(besides.first);
                auto left_follows = routing_map->following(left);
                if (left_follows.empty())
                {
                    auto left_tra = std::make_pair(Cline2Points(left.centerline()), lanelet2_ex::Trajectory_dir::LEFT);
                    all_trajectory.push_back(left_tra);
                }
                else
                {
                    for (auto left_follow : left_follows)
                    {
                        auto left_follow_paths = GetFollowsInDistance(map, routing_map, left_follow.id(), max_distance - self_remain.first);
                        auto left_paths = LinkLanes(besides.first, left_follow_paths);
                        auto left_tra = Ids2Lines(map, lanelet2_ex::Trajectory_dir::LEFT, left_paths);
                        all_trajectory.insert(all_trajectory.end(), left_tra.begin(), left_tra.end());
                    }
                }
            }
            if (besides_t.second)
            {
                lanelet::Lanelet right = map->laneletLayer.get(besides.second);
                auto right_follows = routing_map->following(right);
                if (right_follows.empty())
                {
                    auto right_tra = std::make_pair(Cline2Points(right.centerline()), lanelet2_ex::Trajectory_dir::RIGHT);
                    all_trajectory.push_back(right_tra);
                }
                else
                {
                    for (auto right_follow : right_follows)
                    {
                        auto right_follow_paths = GetFollowsInDistance(map, routing_map, right_follow.id(), max_distance - self_remain.first);
                        auto right_paths = LinkLanes(besides.second, right_follow_paths);
                        auto right_tra = Ids2Lines(map, lanelet2_ex::Trajectory_dir::RIGHT, right_paths);
                        all_trajectory.insert(all_trajectory.end(), right_tra.begin(), right_tra.end());
                    }
                }
            }
        }
        else
        {
            lanelet::Ids left_sets, right_sets;
            std::vector<lanelet::Ids> straight_paths;
            left_sets.clear(), right_sets.clear();
            if (besides_t.first)
            {
                left_sets.push_back(besides.first);
            }
            if (besides_t.second)
            {
                right_sets.push_back(besides.second);
            }
            for (auto follow : follows)
            {
                auto follow_paths = GetFollowsInDistance(map, routing_map, follow.id(), max_distance - self_remain.first);
                auto paths = LinkLanes(closest.second, follow_paths);
                auto tra = Ids2Lines(map, lanelet2_ex::Trajectory_dir::STRAIGHT, paths);
                all_trajectory.insert(all_trajectory.end(), tra.begin(), tra.end());
                for (auto follow_path : follow_paths)
                {
                    for (auto lane_id : follow_path)
                    {
                        std::pair<lanelet::Id, lanelet::Id> temp_besides_ids;
                        auto temp_besides = GetSlice(lane_id, map, routing_map, temp_besides_ids);
                        if (temp_besides.first)
                        {
                            left_sets.push_back(temp_besides_ids.first);
                        }
                        if (temp_besides.second)
                        {
                            right_sets.push_back(temp_besides_ids.second);
                        }
                    }
                }
            }

            auto left_paths = LinkFollows(map, left_sets);
            auto left_tras = Ids2Lines(map, lanelet2_ex::Trajectory_dir::LEFT, left_paths);
            all_trajectory.insert(all_trajectory.end(), left_tras.begin(), left_tras.end());

            auto right_paths = LinkFollows(map, right_sets);
            auto right_tras = Ids2Lines(map, lanelet2_ex::Trajectory_dir::RIGHT, right_paths);
            all_trajectory.insert(all_trajectory.end(), right_tras.begin(), right_tras.end());

            return all_trajectory;
        }
    }

    std::vector<std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Lan_Atr>>> GetRoutes(const lanelet::LaneletMapUPtr &map, const lanelet::routing::RoutingGraphUPtr &routing_map, const lanelet::Point3d point, const double theta, const double max_distance)
    {
        //推荐搜索距离为车身宽度的一半或者更小
        std::vector<std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Lan_Atr>>> all_routes;
        all_routes.clear();
        lanelet::ConstPoints3d locate_cpoints;
        lanelet::Ids raw_locate_lane_ids;
        
        int closest_num = GetClosestPoints_Lanelets(map, point, theta, false, locate_cpoints, raw_locate_lane_ids);
        //去除车道左右或者前后交接处的临近匹配
        //只保留没有前驱车道的id
        lanelet::Ids start_ids;
        lanelet::Ids locate_lane_ids;
        locate_lane_ids.clear();
        for(auto id:raw_locate_lane_ids)
        {
            lanelet::Lanelet temp_start=map->laneletLayer.get(id);
            auto prevs=routing_map->previous(temp_start,false);
            bool start_flag = true; //whether a lane is a sequence start ,that depends on whether any of its previous  is in the list
            if(prevs.empty())
            {
                start_ids.push_back(id);
            }
            else
            {
                for(auto prev_lane:prevs)
                {
                    if(std::find(raw_locate_lane_ids.begin(),raw_locate_lane_ids.end(),id)!=raw_locate_lane_ids.end())
                    {
                        start_flag=false;
                    }
                }
                if (start_flag)
                {
                    start_ids.push_back(id);
                }
            }
        }
        //相邻车道只保留一条
        std::vector<lanelet::Id> has_besides;
        has_besides.clear();
        for(auto id : start_ids)
        {
            if(std::find(has_besides.begin(),has_besides.end(),id)!=has_besides.end())
            {
                continue;
            }
            locate_lane_ids.push_back(id);
            lanelet::Lanelet temp_lane=map->laneletLayer.get(id);
            auto besides=routing_map->besides(temp_lane);
            for (auto beside : besides)
            {
                has_besides.push_back(beside.id());
            }
        }
        
        
        for (int i = 0; i < locate_lane_ids.size(); i++)
        {
            std::vector<std::pair<lanelet::ConstPoints3d, lanelet2_ex::Lan_Atr>> all_trajectory;
            all_trajectory.clear();
            std::pair<lanelet::Id, lanelet::Id> besides;
            std::pair<bool, bool> besides_t = GetSlice(locate_lane_ids[i], map, routing_map, besides);
            lanelet::Lanelet lane_ = map->laneletLayer.get(locate_lane_ids[i]);
            auto follows = routing_map->following(lane_);
            auto self_remain = GetRemainDistance(point, lane_.centerline()); //first 剩余距离
            if (self_remain.first > max_distance)
            {
                lanelet2_ex::Lan_Atr self_attr;
                self_attr.dir = lanelet2_ex::Trajectory_dir::STRAIGHT;
                self_attr.dis = lanelet::geometry::distanceToCenterline2d(lane_, point.basicPoint2d());
                auto self_tra = std::make_pair(Cline2Points(lane_.centerline()), self_attr);
                all_trajectory.push_back(self_tra);
                if (besides_t.first)
                {
                    lanelet::Lanelet left = map->laneletLayer.get(besides.first);
                    lanelet2_ex::Lan_Atr left_attr;
                    left_attr.dir = lanelet2_ex::Trajectory_dir::LEFT;
                    left_attr.dis = lanelet::geometry::distanceToCenterline2d(left, point.basicPoint2d());
                    auto left_tra = std::make_pair(Cline2Points(left.centerline()), left_attr);
                    all_trajectory.push_back(left_tra);
                }
                if (besides_t.second)
                {
                    lanelet::Lanelet right = map->laneletLayer.get(besides.second);
                    lanelet2_ex::Lan_Atr right_attr;
                    right_attr.dir = lanelet2_ex::Trajectory_dir::RIGHT;
                    right_attr.dis = lanelet::geometry::distanceToCenterline2d(right, point.basicPoint2d());
                    auto right_tra = std::make_pair(Cline2Points(right.centerline()), right_attr);
                    all_trajectory.push_back(right_tra);
                }
                all_routes.push_back(all_trajectory);
                continue;
            }
            else if (self_remain.first < max_distance && follows.empty())
            {
                lanelet2_ex::Lan_Atr self_attr;
                self_attr.dir = lanelet2_ex::Trajectory_dir::STRAIGHT;
                self_attr.dis = lanelet::geometry::distanceToCenterline2d(lane_, point.basicPoint2d());
                auto self_tra = std::make_pair(Cline2Points(lane_.centerline()), self_attr);
                all_trajectory.push_back(self_tra);
                if (besides_t.first)
                {
                    lanelet::Lanelet left = map->laneletLayer.get(besides.first);
                    auto left_follows = routing_map->following(left);
                    if (left_follows.empty())
                    {
                        lanelet2_ex::Lan_Atr left_attr;
                        left_attr.dir = lanelet2_ex::Trajectory_dir::LEFT;
                        left_attr.dis = lanelet::geometry::distanceToCenterline2d(left, point.basicPoint2d());
                        auto left_tra = std::make_pair(Cline2Points(left.centerline()), left_attr);
                        all_trajectory.push_back(left_tra);
                    }
                    else
                    {
                        for (auto left_follow : left_follows)
                        {
                            lanelet2_ex::Lan_Atr left_attr;
                            left_attr.dir = lanelet2_ex::Trajectory_dir::LEFT;
                            left_attr.dis = lanelet::geometry::distanceToCenterline2d(left, point.basicPoint2d());
                            auto left_follow_paths = GetFollowsInDistance(map, routing_map, left_follow.id(), max_distance - self_remain.first);
                            auto left_paths = LinkLanes(besides.first, left_follow_paths);
                            auto left_tra = Ids2LinesWithTag(map, left_attr, left_paths);
                            all_trajectory.insert(all_trajectory.end(), left_tra.begin(), left_tra.end());
                        }
                    }
                }
                if (besides_t.second)
                {
                    lanelet::Lanelet right = map->laneletLayer.get(besides.second);
                    auto right_follows = routing_map->following(right);
                    if (right_follows.empty())
                    {
                        lanelet2_ex::Lan_Atr right_attr;
                        right_attr.dir = lanelet2_ex::Trajectory_dir::RIGHT;
                        right_attr.dis = lanelet::geometry::distanceToCenterline2d(right, point.basicPoint2d());
                        auto right_tra = std::make_pair(Cline2Points(right.centerline()), right_attr);
                        all_trajectory.push_back(right_tra);
                    }
                    else
                    {
                        for (auto right_follow : right_follows)
                        {
                            lanelet2_ex::Lan_Atr right_attr;
                            right_attr.dir = lanelet2_ex::Trajectory_dir::RIGHT;
                            right_attr.dis = lanelet::geometry::distanceToCenterline2d(right, point.basicPoint2d());
                            auto right_follow_paths = GetFollowsInDistance(map, routing_map, right_follow.id(), max_distance - self_remain.first);
                            auto right_paths = LinkLanes(besides.second, right_follow_paths);
                            auto right_tra = Ids2LinesWithTag(map, right_attr, right_paths);
                            all_trajectory.insert(all_trajectory.end(), right_tra.begin(), right_tra.end());
                        }
                    }
                }
            }
            else
            {
                lanelet::Ids left_sets, right_sets;
                std::vector<lanelet::Ids> straight_paths;
                left_sets.clear(), right_sets.clear();
                if (besides_t.first)
                {
                    left_sets.push_back(besides.first);
                }
                if (besides_t.second)
                {
                    right_sets.push_back(besides.second);
                }
                for (auto follow : follows)
                {
                    auto follow_paths = GetFollowsInDistance(map, routing_map, follow.id(), max_distance - self_remain.first);
                    auto paths = LinkLanes(locate_lane_ids[i], follow_paths);
                    lanelet2_ex::Lan_Atr follow_attr;
                    follow_attr.dir = lanelet2_ex::Trajectory_dir::STRAIGHT;
                    follow_attr.dis = lanelet::geometry::distanceToCenterline2d(lane_, point.basicPoint2d());
                    auto tra = Ids2LinesWithTag(map, follow_attr, paths);
                    all_trajectory.insert(all_trajectory.end(), tra.begin(), tra.end());
                    for (auto follow_path : follow_paths)
                    {
                        for (auto lane_id : follow_path)
                        {
                            std::pair<lanelet::Id, lanelet::Id> temp_besides_ids;
                            auto temp_besides = GetSlice(lane_id, map, routing_map, temp_besides_ids);
                            if (temp_besides.first)
                            {
                                left_sets.push_back(temp_besides_ids.first);
                            }
                            if (temp_besides.second)
                            {
                                right_sets.push_back(temp_besides_ids.second);
                            }
                        }
                    }
                }
                if(!left_sets.empty())
                {
                    lanelet2_ex::Lan_Atr left_follow_attr;
                    left_follow_attr.dir = lanelet2_ex::Trajectory_dir::LEFT;
                    left_follow_attr.dis = lanelet::geometry::distanceToCenterline2d(map->laneletLayer.get(left_sets.front()), point.basicPoint2d());
                    auto left_paths = LinkFollows(map, left_sets);
                    auto left_tras = Ids2LinesWithTag(map, left_follow_attr, left_paths);
                    all_trajectory.insert(all_trajectory.end(), left_tras.begin(), left_tras.end());
                }
                if(!right_sets.empty())
                {
                    lanelet2_ex::Lan_Atr right_follow_attr;
                    right_follow_attr.dir = lanelet2_ex::Trajectory_dir::RIGHT;
                    right_follow_attr.dis = lanelet::geometry::distanceToCenterline2d(map->laneletLayer.get(right_sets.front()), point.basicPoint2d());
                    auto right_paths = LinkFollows(map, right_sets);
                    auto right_tras = Ids2LinesWithTag(map, right_follow_attr, right_paths);
                    all_trajectory.insert(all_trajectory.end(), right_tras.begin(), right_tras.end());
                }
            }
            all_routes.push_back(all_trajectory);
        }
        return all_routes;
    }

    unsigned int GetClosestIndexInPath(const std::vector<int> path, const lanelet::Point3d point, const lanelet::LaneletMapUPtr &map)
    {
        if (path.size() <= 1)
        {
            return 0;
        }
        else
        {
            unsigned int index = 0;
            double min_dis = DBL_MAX;
            for (unsigned int i = 0; i < path.size(); i++)
            {
                auto lane = map->laneletLayer.get(path[i]);
                auto temp_dis = lanelet::geometry::distance3d(lane.polygon3d(), point);
                if (temp_dis < min_dis)
                {
                    index = i;
                    min_dis = temp_dis;
                }
            }
            return index;
        }
    }

} //namespace lanelet2_ex