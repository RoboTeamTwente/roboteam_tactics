#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/PrepareKickoffUsTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG PrepareKickoffUsTactic

namespace rtt {

RTT_REGISTER_TACTIC(PrepareKickoffUsTactic);

PrepareKickoffUsTactic::PrepareKickoffUsTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        {}

void PrepareKickoffUsTactic::Initialize() {
    tokens.clear();

    RTT_DEBUGLN("Preparing Kickoff...");

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Position the keeper
    int const KEEPER_ID = RobotDealer::get_keeper();
    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", KEEPER_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        Vector2 lookAtVec = Vector2(0, 0) - Vector2(-4, 0);

        ScopedBB(bb, "_GoToPos")
            .setBool("enterDefenseAreas", true)
            .setDouble("angleGoal", lookAtVec.angle())
            .setDouble("xGoal", -4)
            .setDouble("yGoal", 0)
            .setBool("dribbler", false)
            .setBool("avoidRobots", true)
            .setString("stayOnSide", "ourSide")
            ;

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = KEEPER_ID;
        wd.tree = "rtt_bob/GoToPosAndStay";
        wd.blackboard = bb.toMsg();
        wd.token = unique_id::toMsg(unique_id::fromRandom());

        pub.publish(wd);

        claim_robot(KEEPER_ID);
    }

    std::vector<int> robots = getAvailableRobots();

    if (robots.size() == 0) return;

    // Getter bot
    {
        int const ROBOT_ID = robots.back();
        robots.pop_back();

        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        ScopedBB(bb, "_GetBall")
            .setString("AimAt", "theirgoal")
            .setString("stayOnSide", "ourSide")
            ;

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = ROBOT_ID;
        wd.tree = "rtt_bob/GetBallAndStay";
        wd.blackboard = bb.toMsg();
        wd.token = unique_id::toMsg(unique_id::fromRandom());

        pub.publish(wd);

        claim_robot(ROBOT_ID);
    }

    std::vector<rtt::Vector2> startPositions = {
        {-0.5, 2},
        {-0.5, -2},
        {-2, 2},
        {-2, -2},
    };

    for (auto const ROBOT_ID : robots) {
        // Get the first position
        auto const START_POS = startPositions.front();
        // Erase it
        startPositions.erase(startPositions.begin());

        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        Vector2 lookAtVec = Vector2(0, 0) - START_POS;

        ScopedBB(bb, "_GoToPos")
            .setBool("isKeeper", false)
            .setDouble("angleGoal", lookAtVec.angle())
            .setDouble("xGoal", START_POS.x)
            .setDouble("yGoal", START_POS.y)
            .setBool("dribbler", false)
            .setBool("avoidRobots", true)
            .setString("stayOnSide", "ourSide")
            ;

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = ROBOT_ID;
        wd.tree = "rtt_bob/GoToPosAndStay";
        wd.blackboard = bb.toMsg();
        wd.token = unique_id::toMsg(unique_id::fromRandom());

        pub.publish(wd);

        claim_robot(ROBOT_ID);
    }
}

bt::Node::Status PrepareKickoffUsTactic::Update() {
    // PrepareKickoffUsTactic is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    return bt::Node::Status::Running;
}


} // rtt
