#include <iostream>

#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"

#define ROS_LOG_NAME "skills.ShootAtGoalV2"

namespace rtt {
    
RTT_REGISTER_SKILL(ShootAtGoalV2);
    
ShootAtGoalV2::ShootAtGoalV2(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {
	ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "New ShootAtGoalV2");
}

bool isKicking = false;

void ShootAtGoalV2::Initialize(){
	ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Initializing ShootAtGoalV2 : " << blackboard->toString().c_str());

	if(getExtendedState() != RefState::DO_PENALTY){
		ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Warning : " << ROS_LOG_NAME << " is specifically designed for RefState::DO_PENALTY");
	}


}

bt::Node::Status ShootAtGoalV2::Update() {
	// Get the last world
    const roboteam_msgs::World world = LastWorld::get();
	// === Get the ID of the kicker ===
    int robotID = blackboard->GetInt("ROBOT_ID");
    roboteam_msgs::WorldRobot bot;
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        bot = *findBot;
    } else {
		ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "could not find robot with id " << robotID);
        return Status::Failure;
    }
	// =================================

	// Get the position of our kicker
    const Vector2 ownPos(bot.pos);

    partition.reset();
    partition.calculatePartition(world, ownPos);
//    partition.draw();

    auto largest = partition.largestOpenSection();
    if (!largest) {
        return bt::Node::Status::Invalid;
    }
    
    Vector2 target = largest->center;
    double targetAngle = (target - ownPos).angle();

	// aimer = GetBall skill. If GetBall is not initialized
    if (!aimer) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();

        private_bb->SetInt("ROBOT_ID", bot.id);
		private_bb->SetBool("SAGV2_GetBall_passOn", true);
		private_bb->SetBool("passOn", true);
		private_bb->SetBool("SAGV2_GetBall_enterDefenseAreas", true);
		private_bb->SetBool("enterDefenseAreas", true);
    	aimer = std::make_unique<GetBall>("SAGV2_GetBall", private_bb);

		aimer->Initialize();

    }


	isKicking = true;

	private_bb->SetDouble("SAGV2_GetBall_targetAngle", targetAngle);
    private_bb->SetDouble("targetAngle", targetAngle);

	ROS_INFO_STREAM_THROTTLE_NAMED(1, ROS_LOG_NAME, "ROBOT_ID=" << blackboard->GetInt("ROBOT_ID") << ", Robot=" << bot.id << ", Kicking=" << (isKicking ? "Yes" : "No") << ", Target=" << target);
//	ROS_INFO_STREAM_NAMED("ShootAtGoalv2", "Current blackboard: " << blackboard->toString().c_str());

	return aimer->Update();

}
    
}
