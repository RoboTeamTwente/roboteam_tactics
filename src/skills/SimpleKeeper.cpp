#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/SimpleKeeper.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG SimpleKeeper

namespace rtt {

RTT_REGISTER_SKILL(SimpleKeeper);


SimpleKeeper::SimpleKeeper(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb)
        , goToPos("", private_bb) { }

bt::Node::Status SimpleKeeper::Update() {
    // Get the last world information and some blackboard info
    roboteam_msgs::World world = LastWorld::get();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    robotID = blackboard->GetInt("ROBOT_ID");

    double distanceFromGoal;
    double acceptableDeviation;
    double dribblerDist;
    std::string fieldType = GetString("fieldType");
    if (fieldType == "small") {
        distanceFromGoal = 0.4;
        acceptableDeviation = 0.7;
        dribblerDist = 1.0;
    } else if (fieldType == "large") {
        distanceFromGoal = 0.7;
        acceptableDeviation = 1.5;
        dribblerDist = 2.0;
    }

    Vector2 ballPos(world.ball.pos);
    Vector2 goalPos;
    bool ourSide;
    if (HasBool("ourSide")) {
        ourSide = GetBool("ourSide");
        if (ourSide) {
            goalPos = LastWorld::get_our_goal_center();
        } else {
            goalPos = LastWorld::get_their_goal_center();
        }
    } else {
        ourSide = true;
        goalPos = LastWorld::get_our_goal_center();
    }

    double angle = (ballPos - goalPos).angle();

    Vector2 targetPos(distanceFromGoal, 0.0);
    targetPos = targetPos.rotate(angle);
    targetPos = goalPos + targetPos;

    if (fabs(ballPos.x) > (field.field_length/2 - 0.2) || fabs(ballPos.y) > (field.field_width/2 - 0.2)) {
        targetPos = goalPos - Vector2(distanceFromGoal, 0.0) * signum(goalPos.x);
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", (Vector2(0.0, 0.0)-goalPos).angle());
        goToPos.Update();
        return Status::Running;
    } else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("receiveBallAtX", targetPos.x);
        private_bb->SetDouble("receiveBallAtY", targetPos.y);
        private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
        private_bb->SetDouble("dribblerDist", dribblerDist);

        return receiveBall.Update();
    }
}

} // rtt