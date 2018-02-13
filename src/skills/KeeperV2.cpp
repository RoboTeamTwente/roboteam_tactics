#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/KeeperV2.h"

#include "roboteam_utils/LastWorld.h"

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/World.h"

#define RTT_CURRENT_DEBUG_TAG KeeperV2

namespace rtt {

RTT_REGISTER_SKILL(KeeperV2);

KeeperV2::KeeperV2(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {}

void KeeperV2::Initialize() {
    marginFromGoal = 0.08;
    if (HasDouble("marginFromGoal")) {
        marginFromGoal = GetDouble("marginFromGoal");
    }
    double L = 0.5;
    if (HasDouble("distanceFromGoal")) {
        L = GetDouble("distanceFromGoal");
    }
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    W = field.goal_width;
    double circR = L/2 + W*W/(8*L);
    double circX = L/2 - W*W/(8*L);
    double circA = acos(circX/circR);

    goalPos = LastWorld::get_our_goal_center();
    circCenter = Vector2(goalPos.x + circX + marginFromGoal, 0.0);
    blockCircle = Arc(circCenter, circR, circA-M_PI, M_PI-circA);
    ROS_INFO_STREAM(circCenter.x << ", " << circCenter.y << ", " << circA);
}

Vector2 KeeperV2::computeBlockPoint(Vector2 defendPos) {

    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = blockCircle.intersectionWithLine(goalPos,defendPos);
    
    Vector2 blockPos;
    if (intersections.first) {
        blockPos = *intersections.first;
    } else if (intersections.second) {
        ROS_INFO_STREAM("second");
        blockPos = *intersections.second;
    } else {
        blockPos = Vector2(goalPos.x + marginFromGoal, W/2*signum(defendPos.y));
    }

    // if ((defendPos - goalPos).length() < (blockPos - goalPos).length()) {
    //     blockPos = Vector2()
    // }

    return blockPos;
}

Vector2 KeeperV2::computeBlockPoint2(Vector2 defendPos) {
    double a1 = (defendPos - Vector2(marginFromGoal, W/2) - goalPos).angle();
    double a2 = (defendPos - Vector2(marginFromGoal, -W/2) - goalPos).angle();
    double a_mean = a1 + cleanAngle(a2 - a1)/2;
    double dist = (defendPos - goalPos).length();

    Vector2 blockPos = defendPos - Vector2(dist/2, 0.0).rotate(a_mean);
    if (fabs(blockPos.y) > W/2) {
        Vector2 diff = blockPos - defendPos;
        blockPos = defendPos + diff.scale((fabs(defendPos.y)-W/2) / fabs(diff.y));
    }
    // double ratio = fabs(blockPos.y - defendPos.y)/(W/2);
    // if (ratio > 1.0) {
    //     blockPos = (blockPos-goalPos).scale(1/ratio) + goalPos;
    // }
    if (blockPos.x < marginFromGoal + goalPos.x) {
        blockPos.x = marginFromGoal + goalPos.x;
    }
    // Vector2 blockPos(defendPos - blockVec);
    return blockPos;
}

bt::Node::Status KeeperV2::Update() {

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    if (ballPos.x < goalPos.x + marginFromGoal/2) {
        ballPos.x = goalPos.x + marginFromGoal/2;
    }
    Vector2 blockPoint = computeBlockPoint2(ballPos);

    drawer.setColor(0, 0, 255);
    drawer.drawPoint("blockPoint", blockPoint);
    drawer.drawPoint("circCenter", circCenter);
    // drawer.drawArc("blockCircle",blockCircle);
    drawer.drawLine("linedenk", goalPos, ballPos - goalPos);

    return Status::Running;
}

} // rtt