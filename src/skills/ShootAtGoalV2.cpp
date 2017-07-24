#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include <iostream>


namespace rtt {
    
RTT_REGISTER_SKILL(ShootAtGoalV2);
    
ShootAtGoalV2::ShootAtGoalV2(std::string name, bt::Blackboard::Ptr blackboard) 
        : Skill(name, blackboard) {

}

bt::Node::Status ShootAtGoalV2::Update() {
    const roboteam_msgs::World world = LastWorld::get();

    int robotID = blackboard->GetInt("ROBOT_ID");
    roboteam_msgs::WorldRobot bot;
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        bot = *findBot;
    } else {
        ROS_WARN("ShootAtGoalV2 could not find robot");
        return Status::Failure;
    }

    const Vector2 ownPos(bot.pos);
    // const double orientation = bot->angle;
    
    partition.reset();
    partition.calculatePartition(world, ownPos);
    partition.draw();
    auto largest = partition.largestOpenSection();
    if (!largest) {
        return bt::Node::Status::Invalid;
    }
    
    Vector2 target = largest->center;
    double targetAngle = (target - ownPos).angle();
    
    if (!aimer) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        private_bb->SetInt("ROBOT_ID", bot.id);
        private_bb->SetBool("SAGV2_GetBall_passOn", !GetBool("waitForDO_PENALTY", false));
    	aimer = std::make_unique<GetBall>("SAGV2_GetBall", private_bb);
    }
    if (GetBool("waitForDO_PENALTY", false)) {
    	ROS_INFO("still waiting...");
    	private_bb->SetBool("SAGV2_GetBall_passOn", LastRef::getState() == RefState::DO_PENALTY);
    }
    private_bb->SetDouble("SAGV2_GetBall_targetAngle", targetAngle);
    private_bb->SetDouble("targetAngle", targetAngle);

    return aimer->Update();

}
    
}
