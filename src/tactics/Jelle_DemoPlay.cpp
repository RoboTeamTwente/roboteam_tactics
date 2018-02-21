#include <sstream>

// #include "roboteam_msgs/RoleDirective.h"
// #include "roboteam_utils/Vector2.h"
// #include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/Jelle_DemoPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
// #include "roboteam_msgs/RobotCommand.h"
// #include "roboteam_utils/LastRef.h"
// #include "roboteam_tactics/utils/FeedbackCollector.h"

#define RTT_CURRENT_DEBUG_TAG Jelle_DemoPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jelle_DemoPlay);

Jelle_DemoPlay::Jelle_DemoPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Jelle_DemoPlay::Initialize() {
    failed = false;
    succeeded = false;
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    auto robots = getAvailableRobots();
    if (robots.size() == 0) {
        RTT_DEBUGLN("No robots left to claim!");
        failed = true;
        return;
    }

    Keeper_id = *get_robot_closest_to_our_goal(robots);
    if (!Keeper_id) {
    	failed = true;
    	return;
    }
    claim_robot(Keeper_id);
    std::cout << "DemoKeeperNoKicker role assigned to: " << Keeper_id << "\n";

    
    robots = getAvailableRobots();
    if (robots.size() == 0) {
        RTT_DEBUGLN("No robots left to claim!");
        failed = true;
        return;
    }
    Attacker_id = *get_robot_closest_to_ball(robots);
    if (!Attacker_id) {
        failed = true;
        return;
    }
    claim_robot(Attacker_id);
    std::cout << "DemoAttacker role assigned to: " << Attacker_id << "\n";


    bt::Blackboard bb1;
    bb1.SetInt("ROBOT_ID", Keeper_id);
    bb1.SetInt("KEEPER_ID", Keeper_id);

    bt::Blackboard bb2;
    bb2.SetInt("ROBOT_ID", Attacker_id);
    bb2.SetInt("KEEPER_ID", Keeper_id);

    // rtt::ScopedBB(bb, "GetBallTest_A")
    //     .setString("aimAt", "ballplacement")
    //     .setBool("enterDefenseAreas", true)
    //     .setBool("aimAwayFromTarget", true)
    //     .setDouble("aimAtBallplacement_x", endPos.x)
    //     .setDouble("aimAtBallplacement_y", endPos.y)
    //     ;

    // rtt::ScopedBB(bb, "BallPlacementTest_A")
    //     .setDouble("xPlace", endPos.x)
    //     .setDouble("yPlace", endPos.y)
    //     .setBool("enterDefenseAreas", true)
    //     ;

    // roboteam_msgs::RoleDirective rd1;
    rd1.robot_id = Keeper_id;
    rd1.tree = "rtt_jelle/DemoKeeperNoKicker";
    rd1.blackboard = bb1.toMsg();

    token = unique_id::fromRandom();
    rd1.token = unique_id::toMsg(token);

    pub.publish(rd1);

    // roboteam_msgs::RoleDirective rd2;
    rd2.robot_id = Attacker_id;
    if(HasBool("onlyKeeper") && GetBool("onlyKeeper")){
        rd2.tree = "rtt_jelle/SleepRole";
    } else {
        rd2.tree = "rtt_jelle/DemoAttacker";
    }
    rd2.blackboard = bb2.toMsg();

    token = unique_id::fromRandom();
    rd2.token = unique_id::toMsg(token);

    pub.publish(rd2);
}

bt::Node::Status Jelle_DemoPlay::Update() {
    if (!failed && (!getTeamBot(Keeper_id,true) || !getTeamBot(Attacker_id,true))){
    //     auto pub2 = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    //     for(int i = 1; i<200; i++){
    //     pub2.publish(stop_command(Keeper_id));
    //     pub2.publish(stop_command(Attacker_id));
    // }
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        rd1.tree = "rtt_dennis/HaltRole";
        pub.publish(rd1);
        rd2.tree = "rtt_dennis/HaltRole";
        pub.publish(rd2);
        release_robot(Keeper_id);
        release_robot(Attacker_id);
        failed = true;
    }

    if (failed) {
        return Status::Failure;
    }

    // if (succeeded) {
    //     return Status::Success;
    // }

    // if (feedbacks.find(token) != feedbacks.end()) {
    //     Status status = feedbacks.at(token);
    //     if (status == Status::Success) {
    //         // Yay!
    //         return Status::Success;
    //     } else if (status == Status::Running) {
    //         // Carry on
    //     } else {
    //         // Oh shit
    //         return Status::Failure;
    //     }
    // }

    return bt::Node::Status::Running;
}


} // rtt

