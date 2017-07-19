#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/BallPlacementUsPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"

#define RTT_CURRENT_DEBUG_TAG BallPlacementUsPlay

namespace rtt {

RTT_REGISTER_TACTIC(BallPlacementUsPlay);

BallPlacementUsPlay::BallPlacementUsPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BallPlacementUsPlay::Initialize() {
    failed = false;
    succeeded = false;
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    auto robots = getAvailableRobots();

    if (robots.size() == 0) {
        RTT_DEBUGLN("No robots left to claim!");
        failed = true;
        return;
    }

    Vector2 const endPos = LastRef::get().designated_position;
    Vector2 const ballPos = LastWorld::get().ball.pos;
    Vector2 const restPos = endPos + (ballPos - endPos).normalize() * 0.3;
    double const lookAng = (endPos - ballPos).angle();

    if ((endPos - ballPos).length() <= MAX_DISTANCE_FROM_END_POINT) {
        succeeded = true;
        return;
    }

    auto const ROBOT_ID = get_robot_closest_to_ball(robots);
    if (!ROBOT_ID) {
    	failed = true;
    	return;
    }
    claim_robot(*ROBOT_ID);

    std::cout << "[BallPlacement] Placing ball at: " << endPos << "\n";

    bt::Blackboard bb;
    bb.SetInt("ROBOT_ID", *ROBOT_ID);
    bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

    rtt::ScopedBB(bb, "GetBall_")
        .setDouble("targetAngle", lookAng)
        .setBool("passOn", false)
        .setBool("enterDefenseAreas", true)
        ;

    rtt::ScopedBB(bb, "Dribble_")
        .setDouble("xGoal", endPos.x)
        .setDouble("yGoal", endPos.y)
        ;

    rtt::ScopedBB(bb, "Sleep_")
        .setInt("ms", 1000)
        ;

    rtt::ScopedBB(bb, "GoToPos_")
        .setDouble("xGoal", restPos.x)
        .setDouble("yGoal", restPos.y)
        ;

    roboteam_msgs::RoleDirective rd;
    rd.robot_id = *ROBOT_ID;
    rd.tree = "rtt_bob/PlaceBall";
    rd.blackboard = bb.toMsg();

    token = unique_id::fromRandom();
    rd.token = unique_id::toMsg(token);

    pub.publish(rd);
}

bt::Node::Status BallPlacementUsPlay::Update() {
    // BallPlacementUsPlay is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    
    // Vector2 refData = LastRef::get().designated_position;

    // std::cout << "Designated position: " << refData << "\n";
    
    if (failed) {
        return Status::Failure;
    }

    if (succeeded) {
        return Status::Success;
    }

    if (feedbacks.find(token) != feedbacks.end()) {
        Status status = feedbacks.at(token);
        if (status == Status::Success) {
            // Yay!
            return Status::Success;
        } else if (status == Status::Running) {
            // Carry on
        } else {
            // Oh shit
            return Status::Failure;
        }
    }

    return bt::Node::Status::Running;
}


} // rtt

