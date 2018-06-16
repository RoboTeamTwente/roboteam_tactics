#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Bob_ChipoffAtGoalPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define ROS_LOG_NAME "plays.Bob_COAGPlay"

namespace rtt {

RTT_REGISTER_TACTIC(Bob_ChipoffAtGoalPlay);

Bob_ChipoffAtGoalPlay::Bob_ChipoffAtGoalPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard) {}

void Bob_ChipoffAtGoalPlay::Initialize() {
    failImmediately = false;

    std::vector<int> robots = getAvailableRobots();

    if (robots.size() < 1) {
        ROS_DEBUG_NAMED(ROS_LOG_NAME, "Less than one robot detected; cannot use play!");
        failImmediately = true;
        return;
    }

    {
        auto takerID = get_robot_closest_to_ball(robots);
        if (!takerID) {
            failImmediately = true;
            return;
        }
        taker.robot_id = *takerID;
    }

    claim_robot(taker.robot_id);

    int keeperID = RobotDealer::get_keeper();

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", taker.robot_id);
        bb.SetInt("KEEPER_ID", keeperID);
        bb.SetBool("GetBall_A_chipOn", true);

        // ScopedBB(bb, "GetBall_")
        //     .setString("aimAt", "theirgoal")
        //     .setBool("passOn", false)
        //     .setString("stayOnSide", "ourSide")
        //     ;

        // Create message
        taker.tree = "rtt_jim/KickOffTaker";
        taker.blackboard = bb.toMsg();
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, bb.toTestX());


        // Add random token and save it for later
        taker.token = unique_id::toMsg(unique_id::fromRandom());

        // Send to rolenode
        pub.publish(taker);
    }
}

bt::Node::Status Bob_ChipoffAtGoalPlay::Update() {
    if (failImmediately) return Status::Failure;

    auto feedbackIt = feedbacks.find(unique_id::fromMsg(taker.token));
    if (feedbackIt != feedbacks.end()) {
        if (feedbackIt->second == bt::Node::Status::Success) {
            ROS_INFO_NAMED(ROS_LOG_NAME, "Bob_ChipoffAtGoalPlay return success");
        }
        if (feedbackIt->second == bt::Node::Status::Failure) {
            ROS_INFO_NAMED(ROS_LOG_NAME, "Bob_ChipoffAtGoalPlay return failure");
        }
        return feedbackIt->second;
    }



    return bt::Node::Status::Running;
}

} // rtt
