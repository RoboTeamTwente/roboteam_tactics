#include <cmath>

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/NaiveBlockGoal.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG NaiveBlockGoal

namespace rtt {

using namespace roboteam_utils;

// TODO: Hardcoded!
const double GOAL_AREA_WIDTH = 2.5 * 0.8;
// Distance from front of goal area to goal
const double GOAL_AREA_LENGTH = 1 * 0.8;

bool isBallInGoalArea() {
    Vector2 ballPos(LastWorld::get().ball.pos);
    const double FIELD_LENGTH = LastWorld::get_field().field_length;
    if (get_our_field_side() == "left") {
        if (ballPos.x > -FIELD_LENGTH / 2
                && ballPos.x < -FIELD_LENGTH / 2 + GOAL_AREA_LENGTH
                && ballPos.y > -GOAL_AREA_WIDTH / 2
                && ballPos.y < GOAL_AREA_WIDTH / 2) {
           return true;
        }
    } else {
        if (ballPos.x < FIELD_LENGTH / 2
                && ballPos.x > FIELD_LENGTH / 2 - GOAL_AREA_LENGTH
                && ballPos.y > -GOAL_AREA_WIDTH / 2
                && ballPos.y < GOAL_AREA_WIDTH / 2) {
           return true;
        }
    }

    return false;
}

NaiveBlockGoal::NaiveBlockGoal(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , goToPos(n, "", private_bb) {
        	pubNaiveBlockGoal = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
}

bt::Node::Status NaiveBlockGoal::Update() {
    using namespace roboteam_utils;

    const double FIELD_LENGTH = LastWorld::get_field().field_length;

    auto ballPos = Vector2(LastWorld::get().ball.pos);
    Vector2 goalPos = LastWorld::get_our_goal_center();
    Vector2 minVec;

    // if (our_field_side == "left") {
        // goalPos = Vector2(FIELD_LENGTH / -2, 0);
    // } else {
        // goalPos = Vector2(FIELD_LENGTH / 2, 0);
    // }

    std::string our_field_side = get_our_field_side();
    auto ballVec = ballPos - goalPos;

    double padding = 0.10;
    if (our_field_side == "left" && ballPos.x <= -FIELD_LENGTH / 2) {
        minVec.x = -FIELD_LENGTH / 2 + padding;
        minVec.y = ballPos.y;
    } else if (our_field_side == "right" && ballPos.x >= FIELD_LENGTH / 2) {
        minVec.x = FIELD_LENGTH / 2 - padding;
        minVec.y = ballPos.y;
    } else if (isBallInGoalArea()) {
        minVec = ballVec * 0.9 + goalPos;
    } else {
        Vector2 horVec;
        Vector2 vertVec;

        vertVec = ballVec.normalize();
        vertVec = vertVec * std::abs(1 / vertVec.x);
        vertVec = vertVec * (GOAL_AREA_LENGTH);

        horVec = ballVec.normalize();
        horVec = horVec * std::abs(1 / horVec.y);
        horVec = horVec * (GOAL_AREA_WIDTH / 2);
        
        if (!horVec.real()) {
            minVec = vertVec;
        } else if (!vertVec.real()) {
            minVec = horVec;
        } else if (vertVec.length() < horVec.length()) {
            minVec = vertVec;
        } else {
            minVec = horVec;
        }
        
        RTT_DEBUG("Goal area width: %f\n", GOAL_AREA_WIDTH);
        RTT_DEBUG("Ball vec: %f %f\n", ballVec.x, ballVec.y);
        RTT_DEBUG("Hor vec: %f %f\n", horVec.x, horVec.y);
        RTT_DEBUG("Vert vec: %f %f\n", vertVec.x, vertVec.y);
        RTT_DEBUG("Min vec: %f %f\n", minVec.x, minVec.y);

        minVec = minVec + goalPos;

        // If not real, stand in the center of the goal
        if (!minVec.real()) {
            if (our_field_side == "left") {
                minVec.x = -FIELD_LENGTH / 2;
            } else {
                minVec.x = FIELD_LENGTH / 2;
            }
            minVec.y = 0;
        }
    }
    
    if (minVec.y > GOAL_AREA_WIDTH / 2) minVec.y = GOAL_AREA_WIDTH / 2;
    if (minVec.y < -GOAL_AREA_WIDTH / 2) minVec.y = -GOAL_AREA_WIDTH / 2;

    // Always face the ball
    double angle = 0;
    { 
        auto possibleRobot = lookup_our_bot(blackboard->GetInt("ROBOT_ID"));
        if (possibleRobot) {
            auto robot = *possibleRobot;
            angle = (ballPos - Vector2(robot.pos)).angle();
        }
    }
    
    private_bb->SetInt("ROBOT_ID", blackboard->GetInt("ROBOT_ID"));
    private_bb->SetDouble("xGoal", minVec.x);
    private_bb->SetDouble("yGoal", minVec.y);
    private_bb->SetDouble("angleGoal", angle);
    private_bb->SetBool("endPoint", true);
    goToPos.Update();

    return Status::Running;
}

} // rtt
