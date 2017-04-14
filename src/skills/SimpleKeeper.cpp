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
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG SimpleKeeper

namespace rtt {

RTT_REGISTER_SKILL(SimpleKeeper);

SimpleKeeper::SimpleKeeper(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb) { }

bt::Node::Status SimpleKeeper::Update() {
    // Get the last world information and some blackboard info
    roboteam_msgs::World world = LastWorld::get();
    robotID = blackboard->GetInt("ROBOT_ID");

    double distanceFromGoal;
    if (HasDouble("distanceFromGoal")) {
        distanceFromGoal = GetDouble("distanceFromGoal");
    } else {
        distanceFromGoal = 0.4;
    }

    Vector2 ballPos(world.ball.pos);
    Vector2 goalPos = LastWorld::get_our_goal_center();
    double angle = (ballPos - goalPos).angle();

    Vector2 targetPos(distanceFromGoal, 0.0);
    targetPos = targetPos.rotate(angle);
    if (goalPos.x < 0) {
        targetPos = goalPos + targetPos;
    } else {
        targetPos = goalPos - targetPos;
    }

    if (ballPos.y < -0.8 || ballPos.y > 0.8) {
        targetPos = Vector2(distanceFromGoal, 0.0) + goalPos;
    }

    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetDouble("receiveBallAtX", targetPos.x);
    private_bb->SetDouble("receiveBallAtY", targetPos.y);

    return receiveBall.Update();
}

} // rtt
