#include "roboteam_tactics/skills/Block.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/KeeperBlock.h"
#include "roboteam_tactics/utils/DangerFinder.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {
    
RTT_REGISTER_SKILL(KeeperBlock);

KeeperBlock::KeeperBlock(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {
    assert_valid<KeeperBlock>(name);
    target = -1;
    cover_dist = .4;
    reevaluate_target();
}

KeeperBlock::~KeeperBlock() {
    if (impl) impl.reset();
}

boost::optional<roboteam_msgs::WorldRobot> KeeperBlock::select_target() const {
    return most_dangerous_bot();
}

void KeeperBlock::reevaluate_target() {
    boost::optional<roboteam_msgs::WorldRobot> danger = select_target();
    if (!danger || target == danger->id) return;
    
    //ROS_INFO("Target: %d", danger->id);
    
    target = danger->id;
    auto goal = (we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT)[1]; // center

    auto bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("ROBOT_ID", blackboard->GetInt("ROBOT_ID"));
    bb->SetInt("TGT_ID", target);
    bb->SetInt("BLOCK_ID", -1);
    bb->SetDouble("block_x", goal.x);
    bb->SetDouble("block_y", goal.y);
    bb->SetString("block_type", "CIRCLE");
    bb->SetDouble("block_arg", cover_dist);
    bb->SetBool("invert_direction", false);

    impl = std::make_shared<Block>("", bb);
}

bt::Node::Status KeeperBlock::Update() {
    if (impl) impl->extra_update();
    reevaluate_target();
    if (!impl) return bt::Node::Status::Running;
    bt::Node::Status stat = impl->Update();
    // ROS_INFO("Block Status: %s", describe_status(stat).c_str());
    // Keeping is never done, unless something failed elsewhere.
    return stat == bt::Node::Status::Invalid ? stat : bt::Node::Status::Running;
}
    
}
