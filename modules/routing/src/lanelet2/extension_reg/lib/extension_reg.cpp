#include <extension_reg.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

namespace extension_reg
{
    //for Signal
    Signal::Signal(lanelet::Id id,const lanelet::Point3d& point):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_point",{point}});
        signal_type= point.attributeOr("signal_type",0);
        signal_id= point.attributeOr("signal_id",0);
    }

    lanelet::ConstPoint3d Signal::point()
    {
        return getParameters<lanelet::ConstPoint3d>("refer_point").front();
    }

    uint64_t Signal::getType()
    {
        return signal_type;
    }

    uint64_t Signal::getId()
    {
        return signal_id;
    }

    void Signal::setPoint(const lanelet::Point3d& point)
    {
        parameters()["refer_point"]={point};
        signal_type= point.attributeOr("signal_type",0);
        signal_id= point.attributeOr("signal_id",0);
    }


    //for CrossRoad
    CrossRoad::CrossRoad(lanelet::Id id,const lanelet::Area& area):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_area",{area}});
        crossroad_id= area.attributeOr("crossroad_id",0);
    }

    lanelet::ConstArea CrossRoad::area()
    {
        return getParameters<lanelet::ConstArea>("refer_area").front();
    }

    uint64_t CrossRoad::getId()
    {
        return crossroad_id;
    }

    void CrossRoad::setArea(const lanelet::Area& area)
    {
        parameters()["refer_area"]={area};
        crossroad_id= area.attributeOr("crossroad_id",0);
    }


    //for CrossWalk
    CrossWalk::CrossWalk(lanelet::Id id,const lanelet::Area& area):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_area",{area}});
        crosswalk_id= area.attributeOr("crosswalk_id",0);
    }

    lanelet::ConstArea CrossWalk::area()
    {
        return getParameters<lanelet::ConstArea>("refer_area").front();
    }

    uint64_t CrossWalk::getId()
    {
        return crosswalk_id;
    }

    void CrossWalk::setArea(const lanelet::Area& area)
    {
        parameters()["refer_area"]={area};
        crosswalk_id= area.attributeOr("crosswalk_id",0);
    }


    //for RoadEdge
    RoadEdge::RoadEdge(lanelet::Id id,const lanelet::LineString3d& line):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_line",{line}});
        roadedge_id= line.attributeOr("roadedge_id",0);
        direction= line.attributeOr("dir",0);
        type=line.attributeOr("edge_type",0);
    }

    lanelet::ConstLineString3d RoadEdge::line()
    {
        return getParameters<lanelet::ConstLineString3d>("refer_line").front();
    }

    uint64_t RoadEdge::getId()
    {
        return roadedge_id;
    }

    uint64_t RoadEdge::getDir()
    {
        return direction;
    }

    uint64_t RoadEdge::getType()
    {
        return type;
    }

    void RoadEdge::setLine(const lanelet::LineString3d& line)
    {
        parameters()["refer_line"]={line};
        roadedge_id= line.attributeOr("roadedge_id",0);
        direction= line.attributeOr("dir",0);
        type=line.attributeOr("edge_type",0);
    }


    //for TrafficSign
    TrafficSign::TrafficSign(lanelet::Id id,const lanelet::LineString3d& line):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_line",{line}});
        sign_id= line.attributeOr("sign_id",0);
        type= line.attributeOr(lanelet::AttributeName::Subtype,0);
    }

    lanelet::ConstLineString3d TrafficSign::line()
    {
        return getParameters<lanelet::ConstLineString3d>("refer_line").front();
    }

    uint64_t TrafficSign::getId()
    {
        return sign_id;
    }

    uint64_t TrafficSign::getType()
    {
        return type;
    }

    void TrafficSign::setLine(const lanelet::LineString3d& line)
    {
        parameters()["refer_line"]={line};
        sign_id= line.attributeOr("sign_id",0);
        type= line.attributeOr(lanelet::AttributeName::Subtype,0);
    }


    //for StopLine
    StopLine::StopLine(lanelet::Id id,const lanelet::LineString3d& line):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_line",{line}});
        stopline_id= line.attributeOr("stopline_id",0);
    }

    StopLine::StopLine(lanelet::Id id,const lanelet::LineString3d& line,const lanelet::Points3d& points):
    RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
    {
        parameters().insert({"refer_line",{line}});
        stopline_id= line.attributeOr("stopline_id",0);
        parameters().insert({"refer_signals",toRuleParameters(points)});
    }

    lanelet::ConstLineString3d StopLine::line()
    {
        return getParameters<lanelet::ConstLineString3d>("refer_line").front();
    }

    uint64_t StopLine::getId()
    {
        return stopline_id;
    }

    lanelet::ConstPoints3d StopLine::getSignals()
    {
        return getParameters<lanelet::ConstPoint3d>("refer_signals");
    }


    void StopLine::setLine(const lanelet::LineString3d& line)
    {
        parameters()["refer_line"]={line};
        stopline_id= line.attributeOr("stopline_id",0);
    }

    void StopLine::setSignals(const lanelet::Points3d& points)
    {
        parameters()["refer_signals"]={toRuleParameters(points)};
    }


    #if __cplusplus < 201703L
    // instanciate string in cpp file
    constexpr char Signal::RuleName[];  
    constexpr char CrossRoad::RuleName[];
    constexpr char CrossWalk::RuleName[];
    constexpr char RoadEdge::RuleName[];
    constexpr char TrafficSign::RuleName[];
    constexpr char StopLine::RuleName[];
    #endif

    // this object actually does the registration work for us
    lanelet::RegisterRegulatoryElement<Signal> reg_signal;
    lanelet::RegisterRegulatoryElement<CrossRoad> reg_crossroad;
    lanelet::RegisterRegulatoryElement<CrossWalk> reg_crosswalk;
    lanelet::RegisterRegulatoryElement<RoadEdge> reg_roadedge;
    lanelet::RegisterRegulatoryElement<TrafficSign> reg_trafficsign;
    lanelet::RegisterRegulatoryElement<StopLine> reg_stopline;

}