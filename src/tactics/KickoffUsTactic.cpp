#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/tactics/KickoffUsTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "boost/optional.hpp"

namespace b = boost;

#define RTT_CURRENT_DEBUG_TAG KickoffUsTactic

namespace rtt {

RTT_REGISTER_TACTIC(KickoffUsTactic);

KickoffUsTactic::KickoffUsTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void KickoffUsTactic::Initialize() {
    tokens.clear();
    initFailed = false;
    
    RTT_DEBUGLN("Doing Kickoff!");

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
            .setBool("isKeeper", true)
            .setDouble("angleGoal", lookAtVec.angle())
            .setDouble("xGoal", -4)
            .setDouble("yGoal", 0)
            .setBool("dribbler", false)
            .setBool("avoidRobots", true)
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

    // Find robot closest to ball
    std::vector<int> robots = RobotDealer::get_available_robots();
    auto world = LastWorld::get();
    Vector2 ballPos = world.ball.pos;

    double closestDist = std::numeric_limits<double>::max();
    b::optional<int> closestRobotID;

    for (auto robotID : robots) {
        if (auto bot = lookup_our_bot(robotID)) {
            // Robot exists!
            auto dist = (ballPos - Vector2(bot->pos)).length();
            if (dist < closestDist) {
                closestDist = dist;
                closestRobotID = robotID;
            }
        }
    }

    if (!closestRobotID) {
        initFailed = true;
        return;
    }

    // Kicker bot
    {
        int const ROBOT_ID = *closestRobotID;
        // Remove kicker bot so it won't be selected again
        robots.erase(
                std::remove(robots.begin(), robots.end(), ROBOT_ID),
                robots.end()
                );

        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        ScopedBB(bb, "GetBall_")
            .setString("AimAt", "theirgoal")
            ;

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = ROBOT_ID;
        wd.tree = "rtt_bob/GetBallAndChip";
        wd.blackboard = bb.toMsg();
        wd.token = unique_id::toMsg(unique_id::fromRandom());
        
        pub.publish(wd);

        claim_robot(ROBOT_ID);
    }

    for (auto const ROBOT_ID : robots) {
        Vector2 stayPos;
        if (auto botOpt = lookup_our_bot(ROBOT_ID)) {
            stayPos = botOpt->pos;
        } else {
            // If the robot cannot be found we skip it
            continue;
        }

        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        Vector2 lookAtVec = ballPos - stayPos;

        ScopedBB(bb, "_GoToPos")
            .setBool("isKeeper", false)
            .setDouble("angleGoal", lookAtVec.angle())
            .setDouble("xGoal", stayPos.x)
            .setDouble("yGoal", stayPos.y)
            .setBool("dribbler", false)
            .setBool("avoidRobots", true)
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

bt::Node::Status KickoffUsTactic::Update() {
    if (initFailed) return bt::Node::Status::Failure;

    // KickoffUsTactic is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    return bt::Node::Status::Running;
}


} // rtt

