#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/Positioning.h"
#include "roboteam_tactics/utils/utils.h"
#include <chrono>

#include "roboteam_msgs/World.h"

#include "roboteam_utils/LastWorld.h"

#define RTT_CURRENT_DEBUG_TAG Positioning

namespace rtt {

RTT_REGISTER_SKILL(Positioning);

Positioning::Positioning(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {}

void Positioning::Initialize() {
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");

    opportunityFinder.Initialize("jelle.txt", ROBOT_ID, "theirgoal", 0);
    // starting point is opponents half of the field
    bestPosition = opportunityFinder.computeBestOpportunity(Vector2(3.0,0.0),6.0,9.0);

    // pass first info to GoToPos blackboard already
    private_bb->SetInt("ROBOT_ID", ROBOT_ID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", bestPosition.x);
    private_bb->SetDouble("yGoal", bestPosition.y);

    counter = 1;
    start = now();
}

void Positioning::Terminate(bt::Node::Status s){
        // unclaim position
        ros::param::set("robot" + std::to_string(ROBOT_ID) + "/claimedPosX", -0.0);
        ros::param::set("robot" + std::to_string(ROBOT_ID) + "/claimedPosY", -0.0);
        ROS_INFO_STREAM_NAMED("jelle_test1", "Terminating");
    }

bt::Node::Status Positioning::Update() {
    
    auto elapsedTime = time_difference_milliseconds(start, now());
    // best position is computed once every x milliseconds
    if (elapsedTime.count() >= 100) {

        // Determine boxSize: the size of the area scan for best position
        // This boxSize scales down as I get closer to my previously determined best position
        Vector2 myPos = getTargetPos("robot", ROBOT_ID, true);
        double boxSize = (bestPosition - myPos).length() + 1.0; 

        bool dontGoToPos = false;
        if (counter>10) {
            // every 10 scans, the scan area is large again.
            boxSize = 9.0;
            bestPosition = Vector2(3.0,0.0);
            counter = 1;
            // dont immediately go to this new setpoint, because the large area scan is less precise.
            dontGoToPos = true;
        }
        counter++;

        // Determine new best position, in the neighbourhood of the previously determined best position
        bestPosition = opportunityFinder.computeBestOpportunity(bestPosition,boxSize,boxSize);

        // // See what position robot 2 claimed (for debugging)
        // double botClaimedX = 0;
        // double botClaimedY = 0;
        // ros::param::getCached("robot2/claimedPosX", botClaimedX);
        // ros::param::getCached("robot2/claimedPosY", botClaimedY);
        // ROS_INFO_STREAM("robot: " << 2 << " claimed position: x: " << botClaimedX << ", y: "<< botClaimedY);

        if(!dontGoToPos) {
            // I will wait at a set distance from the best position it computed, otherwise the opponent can react, making the position less good.
            // This waiting position is then in a direction such that the best position will not be blocked from the ball or goal by the opponent as a reaction.
            double waitAtDist = 0.0;
            if (HasDouble("waitAtDistance")) {
                waitAtDist = GetDouble("waitAtDistance");
            }
            roboteam_msgs::World world = LastWorld::get();
            Vector2 ballPos(world.ball.pos);
            // Vector2 vecToBall = ballPos - bestPosition; // Vector from bestPosition to ball
            // Vector2 vecToGoal = getTargetPos("theirgoal",0,true) - bestPosition; // Vector from bestPosition to their goalcenter
            // Vector2 posOffset = Vector2(fabs(vecToGoal.y)-fabs(vecToBall.y),vecToBall.x*signum(vecToBall.y)-vecToGoal.x*signum(vecToGoal.y)).stretchToLength(waitAtDist);
            double angleToGoal = (getTargetPos("theirgoal",0,true) - bestPosition).angle(); // Angle from bestPosition to their goal
            double angleToBall = (ballPos - bestPosition).angle();  // Angle from bestPosition to the ball
            Vector2 posOffset = Vector2(-waitAtDist,0).rotate( cleanAngle(angleToBall - angleToGoal)/2 + angleToGoal );

            // pass new position setpoint to GoToPos blackboard
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