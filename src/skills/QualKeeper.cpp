#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/QualKeeper.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#include <cmath>

#define RTT_CURRENT_DEBUG_TAG QualKeeper

namespace rtt {

RTT_REGISTER_SKILL(QualKeeper);

QualKeeper::QualKeeper(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {

}


bt::Node::Status QualKeeper::Update() {
    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    int robotID = blackboard->GetInt("ROBOT_ID");

    roboteam_utils::Vector2 ballPos(world.ball.pos);
    roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();

    double minDistanceToGoal = 0.4;
    double angleToGoal = (ballPos - theirGoalPos).angle();
    double currentDistanceToGoal = minDistanceToGoal / cos(angleToGoal);

    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(currentDistanceToGoal, 0.0).rotate(angleToGoal) + theirGoalPos;
    double targetAngle = 0.5*M_PI;

    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", robotID);
    private_bb->SetDouble("xGoal", targetPos.x);
    private_bb->SetDouble("yGoal", targetPos.y);
    private_bb->SetDouble("angleGoal", targetAngle);
    private_bb->SetBool("avoidRobots", true);
    private_bb->SetBool("dribbler", false);
    goToPos.Update();

	return Status::Running;
}

} // rtt
