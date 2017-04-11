#include "roboteam_tactics/skills/BlockGoal.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/constants.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG BlockGoal

namespace rtt {
    
boost::optional<roboteam_msgs::GeometryFieldSize> BlockGoal::geom;
Arc BlockGoal::topArc; 
Arc BlockGoal::bottomArc;
Section BlockGoal::straightSection;

RTT_REGISTER_SKILL(BlockGoal);

BlockGoal::BlockGoal(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {
    std::string side;
    get_PARAM_OUR_SIDE(side);
    leftSide = side != "left";
    
    if (!geom) {
        geom = LastWorld::get_field();
        
        // Note the '+ M_PI_2l': SSLVision says 0 is the positive y-axis, here it's the positive x-axis,
        // so rotate ccw by a quarter circle.
        if (leftSide) {
            auto top = geom->top_right_penalty_arc;
            auto bottom = geom->bottom_right_penalty_arc;
            topArc = Arc(Vector2(top.center), top.radius, top.a1 + M_PI_2l, top.a2 + M_PI_2l);
            bottomArc = Arc(Vector2(bottom.center), bottom.radius, bottom.a1 + M_PI_2l, bottom.a2 + M_PI_2l);
            straightSection = Section(geom->right_penalty_line.begin, geom->right_penalty_line.end);
        } else {
            auto top = geom->top_left_penalty_arc;
            auto bottom = geom->bottom_left_penalty_arc;
            topArc = Arc(Vector2(top.center), top.radius, top.a1 + M_PI_2l, top.a2 + M_PI_2l);
            bottomArc = Arc(Vector2(bottom.center), bottom.radius, bottom.a1 + M_PI_2l, bottom.a2 + M_PI_2l);
            straightSection = Section(geom->left_penalty_line.begin, geom->left_penalty_line.end);
        }
    }
    
    center = Vector2((leftSide ? 1 : -1) * geom->field_length / 2, 0.0);
}

constexpr double GOAL_LINE_CLEARANCE = .15;

bt::Node::Status BlockGoal::Update() {
    static std::unique_ptr<GoToPos> gtp;
    static Vector2 lastGtpTarget;
    Vector2 tgtPos = LastWorld::get().them.at(GetInt("ROBOT_ID")).pos;
    Vector2 blockPoint = calcBlockPoint(tgtPos);
    
    RTT_DEBUGLN("tgtPos = (%f, %f), initial blockPoint = (%f, %f)", tgtPos.x, tgtPos.y, blockPoint.x, blockPoint.y);
    
    double distanceFromGoalLine = (tgtPos - blockPoint).length() * GOAL_LINE_CLEARANCE;
    blockPoint = blockPoint.lerp(tgtPos, GOAL_LINE_CLEARANCE);
    
    RTT_DEBUGLN("Lerped distance: %f, new pos = (%f, %f)", distanceFromGoalLine, blockPoint.x, blockPoint.y);
    
    double rot = GetDouble("rotationFromOptimal");
    if (rot != 0.0) {
        double rotRadius = geom->top_left_penalty_arc.radius + distanceFromGoalLine;
        blockPoint = blockPoint + Vector2(rotRadius * cosl(rot), rotRadius * sinl(rot));
        RTT_DEBUGLN("Rotating blockPoint: (%f,%f)", blockPoint.x, blockPoint.y);
    }
    
    bt::Blackboard bb;
    bb.SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
    std::string gtpName = "BlockGoalGTP";
    ScopedBB(bb, gtpName).setBool("isKeeper", false)
                         .setDouble("xGoal", blockPoint.x)
                         .setDouble("yGoal", blockPoint.y)
                         .setDouble("angleGoal", (tgtPos - blockPoint).angle())
                         .setBool("dribbler", false)
                         .setBool("avoidRobots", false)
                         .setString("stayOnSide", "");
    if (fabs(blockPoint.dist2(lastGtpTarget)) > .2) {
        gtp = std::make_unique<GoToPos>("BlockGoalGTP", std::make_shared<bt::Blackboard>(bb)); 
    }
    auto gtpStatus = gtp->Update();
    return gtpStatus == bt::Node::Status::Invalid || gtpStatus == bt::Node::Status::Failure ? gtpStatus : bt::Node::Status::Running;
}

Vector2 BlockGoal::calcBlockPoint(const Vector2& target) const {
    double topAngle = (target - topArc.center).angle();
    if (topArc.angleWithinArc(topAngle)) {
        Vector2 p = *topArc.arcPointTowards(topAngle);
        RTT_DEBUGLN("blockPoint on top arc: %f, %f + center", p.x, p.y);
        return p + topArc.center;
    }
    double bottomAngle = (target - bottomArc.center).angle();
    if (bottomArc.angleWithinArc(bottomAngle)) {
        Vector2 p = *bottomArc.arcPointTowards(bottomAngle);
        RTT_DEBUGLN("blockPoint on bottom arc: %f, %f + center", p.x, p.y);
        return p + bottomArc.center;
    }
    RTT_DEBUGLN("blockPoint on straight section");
    Section tgtToGoal(target, center);
    Vector2 blockPoint = tgtToGoal.intersection(straightSection);
    if (!straightSection.pointOnLine(blockPoint)) {
         return straightSection.center;
    }
    return blockPoint;
}

}