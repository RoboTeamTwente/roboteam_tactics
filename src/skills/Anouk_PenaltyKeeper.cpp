#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/skills/Anouk_PenaltyKeeper.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/LastWorld.h"
#include <sstream>
#include "roboteam_utils/Math.h"

#define ROS_LOG_NAME "skills.Anouk_PenaltyKeeper"

namespace rtt {

RTT_REGISTER_SKILL(Anouk_PenaltyKeeper);

    Anouk_PenaltyKeeper::Anouk_PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        {}

Vector2 Anouk_PenaltyKeeper::calculateKeeperPosition(){
    // Get the last world
    roboteam_msgs::World world = LastWorld::get();
    // Get the position of the ball
    Vector2 ballPos(world.ball.pos);


    // === Get their robot that is the closest to the ball === //

    int oppKicker = -1; // Holds the kicker ID
    double oppClosestDistance = 9999; // Holds the closest distance

    for(size_t i = 0; i < world.them.size(); i++){
        // Get the distance between the ball and the current opponent
        double distanceToBall = (Vector2(world.them.at(i).pos) - ballPos).length();
        // If this distance is closer than previous distances, store it
        if(distanceToBall < oppClosestDistance ){
            oppClosestDistance = distanceToBall;
            oppKicker = i;
        }
    }

    Vector2 oppKickerPos = Vector2(world.them.at(oppKicker).pos);

    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robot closest to ball : " << oppKicker);

    // ======================================================= //

    double angleBallOppKicker = (oppKickerPos - ballPos).angle();

    keeperPosition = Vector2(-1.2, 0).rotate(angleBallOppKicker);
    keeperPosition = KeeperPosition + Vector2(-4.8, 0);

    return keeperPosition;

}

void Anouk_PenaltyKeeper::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    std::stringstream ss;
    print_blackboard(blackboard, ss);
    RTT_DEBUGLN("%s\n", ss.str().c_str());

    Vector2 keeperPos = calculateKeeperPosition();

    // Claim the keeper
    const int ROBOT_ID = RobotDealer::get_keeper();
    claim_robots({ROBOT_ID});

    // Fill blackboard with relevant info
    bt::Blackboard bb;

    bb.SetInt("ROBOT_ID", ROBOT_ID);
    private_bb->SetDouble("receiveBallAtX", targetPos.x);
    private_bb->SetDouble("receiveBallAtY", targetPos.y);
    private_bb->SetBool("dontDriveToBall", GetBool("dontDriveToBall"));




    // Create message
    roboteam_msgs::RoleDirective wd;
    wd.robot_id = ROBOT_ID;
    wd.tree = "PenaltyKeeperRole";
    wd.blackboard = bb.toMsg();

    // Send to rolenode
    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    pub.publish(wd);
}

bt::Node::Status Anouk_PenaltyKeeper::Update() {
    // Keeper tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

