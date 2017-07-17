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
    
    ROS_INFO("targetAngle: %f", targetAngle);
    if (!aimer) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        private_bb->SetInt("ROBOT_ID", bot.id);
        private_bb->SetBool("SAGV2_GetBall_passOn", true);
    	aimer = std::make_unique<GetBall>("SAGV2_GetBall", private_bb);
    }
    private_bb->SetDouble("SAGV2_GetBall_targetAngle", targetAngle);
    private_bb->SetDouble("targetAngle", targetAngle);

    return aimer->Update();
    /*
    if (fabs(targetAngle - orientation) < ACCEPTABLE_DEVIATION
    		&& ownPos.dist(Vector2(world.ball.pos))) {
    	bb->SetInt("ROBOT_ID", bot->id);
        ScopedBB(*bb, "SAGV2_kick")
            .setInt("ROBOT_ID", bot->id)
            .setBool("wait_for_signal", false)
            .setDouble("kickVel", KICK_VEL);
        Kick kick("SAGV2_kick", bb);
        ROS_INFO("SAGV2 kicking");
        return kick.Update();
    } else {
    	bb->SetInt("ROBOT_ID", bot->id);
        ScopedBB(*bb, "SAGV2_rotate")
            .setInt("ROBOT_ID", bot->id)
            .setString("center", "ball")
            .setDouble("faceTowardsPosx", target.x)
            .setDouble("faceTowardsPosy", target.y)
            .setDouble("w", 2.0);
        RotateAroundPoint rap("SAGV2_rotate", bb);
        ROS_INFO("SAGV2 rotating");
        auto res = rap.Update();
        return res == bt::Node::Status::Invalid || res == bt::Node::Status::Failure ? res : bt::Node::Status::Running;
    }
   */

}
    
}
