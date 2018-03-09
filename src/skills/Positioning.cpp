#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/Positioning.h"
#include <chrono>

#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG Positioning

namespace rtt {

RTT_REGISTER_SKILL(Positioning);

Positioning::Positioning(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {}

void Positioning::Initialize() {
    start = now();
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");

    opportunityFinder.Initialize("jelle.txt", ROBOT_ID, "theirgoal", 0);
    bestPosition = opportunityFinder.computeBestOpportunity(Vector2(2.25,0.0),4.5,3.0);
    counter = 1;
}

bt::Node::Status Positioning::Update() {
    //
    auto elapsedTime = time_difference_milliseconds(start, now());
    // ROS_INFO_STREAM("elapsedTime: " << elapsedTime.count());

    if (elapsedTime.count() >= 1000) {
        if (counter>10) {
            Initialize();
        }
        counter++;

        opportunityFinder.Initialize("jelle.txt", ROBOT_ID, "theirgoal", 0);
        // double score = opportunityFinder.computeScore(Vector2(world.us.at(i).pos));
        Vector2 myPos = getTargetPos("robot", ROBOT_ID, true);
        double boxSize = (bestPosition - myPos).length() + 1.0;
        bestPosition = opportunityFinder.computeBestOpportunity(bestPosition,boxSize,boxSize);
        start = now();
        ROS_INFO_STREAM("robot: " << ROBOT_ID << " best position: x: " << bestPosition.x << ", y: "<< bestPosition.y);
    }
    private_bb->SetInt("ROBOT_ID", ROBOT_ID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", bestPosition.x);
    private_bb->SetDouble("yGoal", bestPosition.y);
    // private_bb->SetDouble("angleGoal", targetAngle);

    goToPos.Update();
    return Status::Running;
}

} // rtt