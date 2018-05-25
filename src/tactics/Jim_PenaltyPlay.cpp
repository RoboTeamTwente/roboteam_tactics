#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_PenaltyPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "unique_id/unique_id.h" 

#define ROS_LOG_NAME "plays.Jim_PenaltyPlay"
#define RTT_CURRENT_DEBUG_TAG Jim_PenaltyPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_PenaltyPlay);

Jim_PenaltyPlay::Jim_PenaltyPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard){}

void Jim_PenaltyPlay::Initialize() {
    tokens.clear();
    success = true;

    roboteam_msgs::World world = LastWorld::get();

    if (getAvailableRobots().size() < 1) {
        ROS_WARN_NAMED(ROS_LOG_NAME, "No robots available, cannot initialize...");
        success = false;
        return;
    }
    
    std::vector<int> robots = getAvailableRobots();
    Vector2 ballPos = Vector2(world.ball.pos);
    auto penaltyTakerID = get_robot_closest_to_point(robots, world, ballPos);

    if (!penaltyTakerID) {
        success = false;
    	return;
    }

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	ROS_DEBUG_NAMED(ROS_LOG_NAME, "Initializing Jim_PenaltyPlay...");


    // =============================
    // Initialize the Ball Getter
    // =============================
    {
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = *penaltyTakerID;
        bt::Blackboard bb;
        claim_robot(*penaltyTakerID);
        activeRobot = *penaltyTakerID;

        bb.SetInt("ROBOT_ID", *penaltyTakerID);
        bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

        bb.SetBool("GetBall_A_passToBestAttacker", true); 
        bb.SetBool("ShootAtGoalV2_A_waitForDO_PENALTY", true);

        // Create message
        rd.tree = "rtt_jim/ShootAtGoalRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    start = rtt::now();
}


bt::Node::Status Jim_PenaltyPlay::Update() {
    if (!success) {
    	return Status::Failure;
    }

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == Status::Success) {
                ROS_DEBUG_NAMED(ROS_LOG_NAME, "Success!");
            }
            if (status == Status::Failure) {
				ROS_DEBUG_NAMED(ROS_LOG_NAME, "Failure!");
            }
            return status;
        }
    }

    return Status::Running;
}

} // rtt
