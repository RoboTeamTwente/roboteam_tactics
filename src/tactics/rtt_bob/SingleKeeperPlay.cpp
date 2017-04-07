#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/rtt_bob/SingleKeeperPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

namespace rtt_bob {

RTT_REGISTER_TACTIC_F(rtt_bob, SingleKeeperPlay);

SingleKeeperPlay::SingleKeeperPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void SingleKeeperPlay::Initialize() {
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    int const KEEPER_ID = RobotDealer::get_keeper();
    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", KEEPER_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        // Create message
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = KEEPER_ID;
        rd.tree = "rtt_bob/BasicKeeperTree";
        rd.blackboard = bb.toMsg();
        rd.token = unique_id::toMsg(unique_id::fromRandom());

        pub.publish(rd);

        claim_robot(KEEPER_ID);
    }

    return;
}

bt::Node::Status SingleKeeperPlay::Update() {
    // BasicKeeperTree never ends, thus this tactics never ends either
    return Status::Running;
}

}

} // rtt

