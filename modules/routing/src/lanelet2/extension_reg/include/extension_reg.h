#ifndef _EXTENSION_REG_H_
#define _EXTENSION_REG_H_

#include <string>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

namespace extension_reg
{

template <typename T>
lanelet::RuleParameters toRuleParameters(const std::vector<T>& primitives) {
  return lanelet::utils::transform(primitives, [](const auto& elem) { return static_cast<lanelet::RuleParameter>(elem); });
}

    class Signal : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<Signal>;
            static constexpr char RuleName[]="Traffic_Signal";
            static Ptr make(lanelet::Id id,const lanelet::Point3d& point)
            {
                return Ptr(new Signal(id,point));
            }

            /**
             * @brief get the basic point for the traffic light
             * @return the basic point as Point3D
             */
            lanelet::ConstPoint3d point();
            /**
             * @brief get the type for the traffic light
             * @return uint_64t as different type
             */
            uint64_t getType();
            /**
             * @brief get the id for the traffic light (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief set the refer_point for the traffic light
             * @return  none  
             */
            void setPoint(const lanelet::Point3d& point);

        private:
            uint64_t signal_id;
            uint64_t signal_type;
            friend class lanelet::RegisterRegulatoryElement<Signal>;
            Signal(lanelet::Id id,const lanelet::Point3d& point);
            explicit Signal(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };
    using Signals=std::vector<Signal>;

    class CrossRoad : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<CrossRoad>;
            static constexpr char RuleName[]="CrossRoad";
            static Ptr make(lanelet::Id id,const lanelet::Area& area)
            {
                return Ptr(new CrossRoad(id,area));
            }

            /**
             * @brief get the basic area for the CrossRoad
             * @return the basic area as lanelet::Area
             */
            lanelet::ConstArea area();
            /**
             * @brief get the id for the CrossRoad (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief set the refer_area for the CrossRoad
             * @return  none  
             */
            void setArea(const lanelet::Area& area);

        private:
            uint64_t crossroad_id;
            friend class lanelet::RegisterRegulatoryElement<CrossRoad>;
            CrossRoad(lanelet::Id id,const lanelet::Area& area);
            explicit CrossRoad(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };

    class CrossWalk : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<CrossWalk>;
            static constexpr char RuleName[]="CrossWalk";
            static Ptr make(lanelet::Id id,const lanelet::Area& area)
            {
                return Ptr(new CrossWalk(id,area));
            }

            /**
             * @brief get the basic area for the CrossWalk
             * @return the basic area as lanelet::Area
             */
            lanelet::ConstArea area();
            /**
             * @brief get the id for the CrossWalk (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief set the refer_area for the CrossWalk
             * @return  none  
             */
            void setArea(const lanelet::Area& area);

        private:
            uint64_t crosswalk_id;
            friend class lanelet::RegisterRegulatoryElement<CrossWalk>;
            CrossWalk(lanelet::Id id,const lanelet::Area& area);
            explicit CrossWalk(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };

    class RoadEdge : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<RoadEdge>;
            static constexpr char RuleName[]="RoadEdge";
            static Ptr make(lanelet::Id id,const lanelet::LineString3d& line)
            {
                return Ptr(new RoadEdge(id,line));
            }

            /**
             * @brief get the basic line for the RoadEdge
             * @return the basic line as lanelet::LineString3d
             */
            lanelet::ConstLineString3d line();
            /**
             * @brief get the id for the RoadEdge (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief get the direction for the RoadEdge (right or left)
             * @return uint_64t as direction
             */
            uint64_t getDir();
            /**
             * @brief get the type for the RoadEdge (the normal is curbstone)
             * @return uint_64t as type
             */
            uint64_t getType();
            /**
             * @brief set the refer_line for the RoadEdge
             * @return  none  
             */
            void setLine(const lanelet::LineString3d& line);

        private:
            uint64_t roadedge_id;
            uint64_t direction;
            uint64_t type;
            friend class lanelet::RegisterRegulatoryElement<RoadEdge>;
            RoadEdge(lanelet::Id id,const lanelet::LineString3d& line);
            explicit RoadEdge(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };


    class TrafficSign : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<TrafficSign>;
            static constexpr char RuleName[]="TrafficSign";
            static Ptr make(lanelet::Id id,const lanelet::LineString3d& line)
            {
                return Ptr(new TrafficSign(id,line));
            }

            /**
             * @brief get the basic line for the TrafficSign
             * @return the basic line as lanelet::LineString3d
             */
            lanelet::ConstLineString3d line();
            /**
             * @brief get the id for the TrafficSign (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief get the type for the TrafficSign
             * @return uint_64t as type
             */
            uint64_t getType();
            /**
             * @brief set the refer_line for the TrafficSign
             * @return  none  
             */
            void setLine(const lanelet::LineString3d& line);

        private:
            uint64_t sign_id;
            uint64_t type;
            friend class lanelet::RegisterRegulatoryElement<TrafficSign>;
            TrafficSign(lanelet::Id id,const lanelet::LineString3d& line);
            explicit TrafficSign(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };
    

    class StopLine : public lanelet::RegulatoryElement
    {
        public:
            using Ptr = std::shared_ptr<StopLine>;
            static constexpr char RuleName[]="StopLine";
            static Ptr make(lanelet::Id id,const lanelet::LineString3d& line,const lanelet::Points3d& points)
            {
                return Ptr(new StopLine(id,line,points));
            }
            static Ptr make(lanelet::Id id,const lanelet::LineString3d& line)
            {
                return Ptr(new StopLine(id,line));
            }

            /**
             * @brief get the basic line for the StopLine
             * @return the basic line as lanelet::LineString3d
             */
            lanelet::ConstLineString3d line();
            /**
             * @brief get the id for the StopLine (but not the regulatory element id in the map)
             * @return uint_64t as id
             */
            uint64_t getId();
            /**
             * @brief get the signals(basic points) link to the StopLine
             * @return lanelet::ConstPoints3d
             */
            lanelet::ConstPoints3d getSignals();
            /**
             * @brief set the refer_line for the StopLine
             * @return  none  
             */
            void setLine(const lanelet::LineString3d& line);
            /**
             * @brief set the refer_signals(basic points) for the StopLine
             * @return  none  
             */
            void setSignals(const lanelet::Points3d& points);

        private:
            uint64_t stopline_id;
            friend class lanelet::RegisterRegulatoryElement<StopLine>;
            StopLine(lanelet::Id id,const lanelet::LineString3d& line);
            StopLine(lanelet::Id id,const lanelet::LineString3d& line,const lanelet::Points3d& points);
            explicit StopLine(const lanelet::RegulatoryElementDataPtr& data): lanelet::RegulatoryElement(data){}
    };

};// extension for customized regulatory elements namespace end

#endif