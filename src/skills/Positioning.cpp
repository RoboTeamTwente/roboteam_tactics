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
    dontGoToPos = false;
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");

    opportunityFinder.Initialize("jelle.txt", ROBOT_ID, "theirgoal", 0);
    // starting point is opponents half of the field
    bestPosition = opportunityFinder.computeBestOpportunity(Vector2(2.25,0.0),4.5,6.0);

    // pass first info to GoToPos blackboard already
    private_bb->SetInt("ROBOT_ID", ROBOT_ID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", bestPosition.x);
    private_bb->SetDouble("yGoal", bestPosition.y);

    counter = 1;
    start = now();
}

bt::Node::Status Positioning::Update() {
    
    auto elapsedTime = time_difference_milliseconds(start, now());
    // ROS_INFO_STREAM("elapsedTime: " << elapsedTime.count());

    // best position is computed once every x milliseconds
    if (elapsedTime.count() >= 100) {
//
        Vector2 myPos = getTargetPos("robot", ROBOT_ID, true);
        double boxSize = (bestPosition - myPos).length() + 1.0;

        dontGoToPos = false;
        if (counter>10) {
            // every 10 scans, the scan area is reset.
            boxSize = 6.0;
            bestPosition = Vector2(2.25,0.0);
            counter = 1;
            // dont immediately go to new setpoint
            dontGoToPos = true;
        }
        counter++;

        bestPosition = opportunityFinder.computeBestOpportunity(bestPosition,boxSize,boxSize);

        double botClaimedX = 0;
        double botClaimedY = 0;

        // ros::param::getCached("robot2/claimedPosX", botClaimedX);
        // ros::param::getCached("robot2/claimedPosY", botClaimedY);
        // ROS_INFO_STREAM("robot: " << 2 << " claimed position: x: " << botClaimedX << ", y: "<< botClaimedY);

        if(!dontGoToPos) {
            // the robot will wait at a set distance from the best position it computed, otherwise the opponent can react, making the position less good.
            // the waiting position is then in the direction perpendicular to the vector from bestposition to goal
            double waitAtDist = 0.0;
            if (HasDouble("waitAtDistance")) {
                waitAtDist = GetDouble("waitAtDistance");
            }
            Vector2 vecToGoal = getTargetPos("theirgoal",0,true) - bestPosition;
            Vector2 posOffset = Vector2(vecToGoal.y,-vecToGoal.x).stretchToLength(waitAtDist) * signum(vecToGoal.y);
            // pass new position setpoint to GoToPos
            private_bb->SetDouble("xGoal", bestPosition.x + posOffset.x);
            private_bb->SetDouble("yGoal", bestPosition.y + posOffset.y);

            // claim chosen position to go to
            ros::param::set("robot" + std::to_string(ROBOT_ID) + "/claimedPosX", bestPosition.x);
            ros::param::set("robot" + std::to_string(ROBOT_ID) + "/claimedPosY", bestPosition.y);
        }

        start = now();
        // ROS_INFO_STREAM("robot: " << ROBOT_ID << " best position: x: " << bestPosition.x << ", y: "<< bestPosition.y);
        
    }
    // private_bb->SetDouble("angleGoal", targetAngle);
    goToPos.Update();
    return Status::Running;
}

} // rtt