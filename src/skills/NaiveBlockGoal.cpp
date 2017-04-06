#include "roboteam_tactics/treegen/LeafRegister.h"
#include <cmath>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/NaiveBlockGoal.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG NaiveBlockGoal

namespace rtt {

using namespace roboteam_utils;

// TODO: Hardcoded!
// const double GOAL_AREA_WIDTH = 0.9;
// Distance from front of goal area to goal
// const double GOAL_AREA_LENGTH = 0.5;

bool isBallInGoalArea(double const GOAL_AREA_LENGTH, double const GOAL_AREA_WIDTH) {
    Vector2 ballPos(LastWorld::get().ball.pos);
    const double FIELD_LENGTH = LastWorld::get_field().field_length;
    if (get_our_side() == "left") {
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

RTT_REGISTER_SKILL(NaiveBlockGoal);

NaiveBlockGoal::NaiveBlockGoal(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) { }

bt::Node::Status NaiveBlockGoal::Update() {
    using namespace roboteam_utils;

    const double FIELD_LENGTH = LastWorld::get_field().field_length;

    auto ballPos = Vector2(LastWorld::get().ball.pos);
    Vector2 minVec;

    // Eew.
    auto field = LastWorld::get_field();

    auto leftPenaltyLineLength = (Vector2(field.left_penalty_line.begin) - Vector2(field.left_penalty_line.end)).length();
    auto const GOAL_AREA_WIDTH = leftPenaltyLineLength + field.top_left_penalty_arc.radius + field.top_right_penalty_arc.radius;

    auto const GOAL_AREA_ARC_RADIUS = field.top_left_penalty_arc.radius;


    // TODO: @Bug this is bad semantics on vision's part
    roboteam_msgs::FieldLineSegment leftPenaltyLine;
    roboteam_msgs::FieldCircularArc topLeftArc;
    roboteam_msgs::FieldCircularArc bottomLeftArc;

    if (field.left_penalty_line.begin.x < field.right_penalty_line.begin.x) {
        leftPenaltyLine = field.left_penalty_line;
    } else {
        leftPenaltyLine = field.right_penalty_line;
    }

    if (field.top_left_penalty_arc.center.x < field.top_right_penalty_arc.center.x) {
        topLeftArc = field.top_left_penalty_arc;
    } else {
        topLeftArc = field.top_right_penalty_arc;
    }

    if (field.bottom_left_penalty_arc.center.x < field.bottom_right_penalty_arc.center.x) {
        bottomLeftArc = field.bottom_left_penalty_arc;
    } else {
        bottomLeftArc = field.bottom_right_penalty_arc;
    }

    auto const leftPenaltyLineYMax = std::max(leftPenaltyLine.begin.y, leftPenaltyLine.end.y);
    auto const leftPenaltyLineYMin = std::min(leftPenaltyLine.begin.y, leftPenaltyLine.end.y);

    double padding = 0.10;

    if (ballPos.x <= -FIELD_LENGTH / 2) {
        minVec.x = -FIELD_LENGTH / 2 + padding;
        minVec.y = ballPos.y;
    } else {
        if (ballPos.y > leftPenaltyLineYMax) {
            drawer.setColor(255, 0, 0);
            drawer.drawLineAbs("yLine", Vector2(-4, leftPenaltyLineYMax), Vector2(-3.25, leftPenaltyLineYMax));

            // Top goal border
            auto topArcCenter = topLeftArc.center;

            drawer.drawPoint("arcCenter", topArcCenter);

            auto ballPosRelativeToArcCenter = ballPos - topArcCenter;

            auto keeperPosRelativeToBottomGoalPole = ballPosRelativeToArcCenter.normalize() * GOAL_AREA_ARC_RADIUS;

            minVec = keeperPosRelativeToBottomGoalPole + topArcCenter;
        } else if (ballPos.y < leftPenaltyLineYMin) {
            drawer.setColor(0, 0, 255);
            drawer.drawLineAbs("yLine", Vector2(-4, leftPenaltyLineYMin), Vector2(-3.25, leftPenaltyLineYMin));

            // Bottom goal border
            auto bottomArcCenter = bottomLeftArc.center;

            drawer.drawPoint("arcCenter", bottomArcCenter);

            auto ballPosRelativeToArcCenter = ballPos - bottomArcCenter;

            auto keeperPosRelativeToBottomGoalPole = ballPosRelativeToArcCenter.normalize() * GOAL_AREA_ARC_RADIUS;

            minVec = keeperPosRelativeToBottomGoalPole + bottomArcCenter;
        } else {
            // In front of the goal
            // Project ball onto penalty line
            
            minVec = ballPos.project(leftPenaltyLine.begin, leftPenaltyLine.end);
        }
    }

    if (minVec.y > GOAL_AREA_WIDTH / 2) minVec.y = GOAL_AREA_WIDTH / 2;
    if (minVec.y < -GOAL_AREA_WIDTH / 2) minVec.y = -GOAL_AREA_WIDTH / 2;

    // Always face the ball
    double angle = 0;
    {
        auto possibleRobot = getWorldBot(blackboard->GetInt("ROBOT_ID"));
        if (possibleRobot) {
            auto robot = *possibleRobot;
            angle = (ballPos - Vector2(robot.pos)).angle();
        }
    }

    private_bb->SetInt("ROBOT_ID", blackboard->GetInt("ROBOT_ID"));
    private_bb->SetDouble("xGoal", minVec.x);
    private_bb->SetDouble("yGoal", minVec.y);
    private_bb->SetDouble("angleGoal", angle);
    private_bb->SetBool("avoidRobots", true);
    goToPos.Update();

    return Status::Running;
}

} // rtt
