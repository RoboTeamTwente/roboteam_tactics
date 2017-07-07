#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/PrepareKeeperPenaltyThem.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG PrepareKeeperPenaltyThem

namespace rtt {

RTT_REGISTER_TACTIC(PrepareKeeperPenaltyThem);

PrepareKeeperPenaltyThem::PrepareKeeperPenaltyThem(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        {}

void PrepareKeeperPenaltyThem::Initialize() {
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Position the keeper
    int const KEEPER_ID = RobotDealer::get_keeper();
    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;

        // The back of the robot should touch the goal line.
        auto left_line = LastWorld::get_field().left_line;
        double goal_line_x = ((left_line.begin.x + left_line.end.x) / 2) + 0.8;

        bb.SetInt("ROBOT_ID", KEEPER_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        Vector2 lookAtVec = Vector2(0, 0) - Vector2(goal_line_x, 0);

        std::cout << goal_line_x << std::endl;

        ScopedBB(bb, "_GoToPos")
            .setBool("enterDefenseAreas", true)
            .setDouble("angleGoal", lookAtVec.angle())
            .setDouble("xGoal", goal_line_x)
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
}

bt::Node::Status PrepareKeeperPenaltyThem::Update() {
    // PrepareKeeperPenaltyThem is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    return bt::Node::Status::Running;
}


} // rtt
