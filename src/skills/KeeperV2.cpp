#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/KeeperV2.h"

#include "roboteam_utils/LastWorld.h"

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/World.h"

#define RTT_CURRENT_DEBUG_TAG KeeperV2

namespace rtt {

RTT_REGISTER_SKILL(KeeperV2);

KeeperV2::KeeperV2(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb) {

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

}

void KeeperV2::Initialize() {
}

// Chooses a position such that both open goal angles are equal as seen from the defendPos
// Then chooses the intersection between this line and an arc between the goal posts
Vector2 KeeperV2::computeBlockPoint(Vector2 defendPos) {

    Vector2 u1 = (goalPos + Vector2(0.0, W/2) - defendPos).normalize(); // unit vector from defendpos to upper goal post
    Vector2 u2 = (goalPos + Vector2(0.0, -W/2) - defendPos).normalize(); // unit vector from defendpos to lower goal post
    double dist = (defendPos - goalPos).length();
    Vector2 blockLineStart = defendPos + (u1 + u2).stretchToLength(dist);

    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = blockCircle.intersectionWithLine(blockLineStart,defendPos);
    
    Vector2 blockPos;
    if (intersections.first) {
        blockPos = *intersections.first;
    } else if (intersections.second) {
        blockPos = *intersections.second;
    } else {
        blockPos = Vector2(goalPos.x + marginFromGoal, W/2*signum(defendPos.y));
    }

    // if defendPos is within the arc, choose position directly behind the defendPos
    if ((defendPos - goalPos).length() - 0.08 < (blockPos - goalPos).length()) {
        blockPos = defendPos + (goalPos - defendPos).stretchToLength(0.1);
    }

    Vector2 distVec = blockPos - defendPos;
    acceptableDeviation = fmax(0.0, (u1.stretchToLength(distVec.length()) - distVec).length() - 0.05 );

    return blockPos;
}

bt::Node::Status KeeperV2::Update() {

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    if (ballPos.x < goalPos.x + marginFromGoal/2) {
        ballPos.x = goalPos.x + marginFromGoal/2;
    }
    Vector2 blockPoint = computeBlockPoint(ballPos);

    drawer.setColor(0, 0, 255);
    drawer.drawPoint("blockPoint", blockPoint);
    drawer.drawPoint("circCenter", circCenter);
    // drawer.drawArc("blockCircle",blockCircle);
    drawer.drawLine("linedenk", goalPos, ballPos - goalPos);

    private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
    private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    private_bb->SetDouble("receiveBallAtX", blockPoint.x);
    private_bb->SetDouble("receiveBallAtY", blockPoint.y);
    private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
    private_bb->SetDouble("marginDeviation", acceptableDeviation);
    private_bb->SetBool("defenderMode", true);
    private_bb->SetBool("setSignal", false);
    private_bb->SetBool("enterDefenseAreas", true);
    private_bb->SetDouble("dribblerDist", 0.5);

    return receiveBall.Tick();
}

} // rtt