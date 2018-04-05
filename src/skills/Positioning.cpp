#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/Positioning.h"
#include <chrono>

#include "roboteam_msgs/World.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG Positioning

namespace rtt {

RTT_REGISTER_SKILL(Positioning);

Positioning::Positioning(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {}

void Positioning::Initialize() {
    robotID = blackboard->GetInt("ROBOT_ID");
    ROS_INFO_STREAM_NAMED("skills.Positioning", "Initialize for robot: " << robotID);
    // passIncoming = false;
    int type = 0;
    if (HasInt("type")) {
        type = GetInt("type");
    }
    if (type == 0) {
    // Attacker
        opportunityFinder.Initialize("jelle.txt", robotID, "theirgoal", 0);
    } else if (type == 1) {
    // Assister/passer
    // WIP: To which robot should be passed? check at every scan which robot?
        // Check which robot:
        

        // opportunityFinder.Initialize("assister.txt", robotID, "robot", 0);
    }

    
    // starting point is opponents half of the field
    bestPosition = opportunityFinder.computeBestOpportunity(Vector2(3.0,0.0),6.0,9.0);

    // pass first info to GoToPos blackboard already
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", bestPosition.x);
    private_bb->SetDouble("yGoal", bestPosition.y);

    counter = 1;
    // counter2 = 0;
    start = now();
}

void Positioning::Terminate(bt::Node::Status s) {
        if (!GetBool("dontUnclaim")) {
        // unclaim position
            ros::param::set("robot" + std::to_string(robotID) + "/claimedPosX", -0.0);
            ros::param::set("robot" + std::to_string(robotID) + "/claimedPosY", -0.0);
            ROS_INFO_STREAM_NAMED("skills.Positioning", "Terminating, unclaim position for robot: " << robotID);
        } else {
            ROS_INFO_STREAM_NAMED("skills.Positioning", "Terminating, keeping claimed pos for robot: " << robotID);
        }
    }

double Positioning::getBallGoalHalfwayAngle(Vector2 testPos) {
    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    double angleToGoal = (LastWorld::get_their_goal_center() - testPos).angle();    // Angle from testPos to their goal
    double angleToBall = (ballPos - testPos).angle();                               // Angle from testPos to the ball
    double halfwayAngle = cleanAngle(angleToBall - angleToGoal)/2 + angleToGoal;
    return halfwayAngle;
}

bt::Node::Status Positioning::Update() {
    

    // if (!passIncoming) {
    //     int passToRobot;
    //     ros::param::getCached("passToRobot", passToRobot);
    //     passIncoming = passToRobot == (int) robotID;
    // } else if (counter2 > 20) {// if a pass is incoming, I should stop repositioning myself and go to the last bestPosition I chose
    //     private_bb->SetDouble("xGoal", bestPosition.x);
    //     private_bb->SetDouble("yGoal", bestPosition.y);
    //     private_bb->SetDouble("angleGoal", getBallGoalHalfwayAngle(bestPosition));
    //     counter2 = 0;
    // } else {
    //     counter2++;
    // }
    

    auto elapsedTime = time_difference_milliseconds(start, now());
    // best position is computed once every x milliseconds
    if (elapsedTime.count() >= 300) { // !passIncoming && 

        // Determine boxSize: the size of the area scan for best position
        // This boxSize scales down as I get closer to my previously determined best position
        Vector2 myPos = getTargetPos("robot", robotID, true);
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

        if(!dontGoToPos) {
            // I will wait at a set distance from the best position it computed, otherwise the opponent can react, making the position less good.
            // This waiting position is then in a direction such that the best position will not be blocked from the ball or goal by the opponent as a reaction.
            double waitAtDist = 0.0;
            if (HasDouble("waitAtDistance")) {
                waitAtDist = GetDouble("waitAtDistance");
            }
            Vector2 posOffset = Vector2(-waitAtDist,0).rotate( getBallGoalHalfwayAngle(bestPosition) );

            // pass new position setpoint to GoToPos blackboard
            private_bb->SetDouble("xGoal", bestPosition.x + posOffset.x);
            private_bb->SetDouble("yGoal", bestPosition.y + posOffset.y);

            // claim chosen position to go to
            ros::param::set("robot" + std::to_string(robotID) + "/claimedPosX", bestPosition.x);
            ros::param::set("robot" + std::to_string(robotID) + "/claimedPosY", bestPosition.y);
        }

        start = now();
        // ROS_INFO_STREAM("robot: " << robotID << " best position: x: " << bestPosition.x << ", y: "<< bestPosition.y);  
    }
    // private_bb->SetDouble("angleGoal", targetAngle);
    goToPos.Update();
    return Status::Running;
}

} // rtt