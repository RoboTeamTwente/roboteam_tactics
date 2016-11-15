#include "roboteam_tactics/skills/KeeperBlock.h"
#include "roboteam_tactics/utils/DangerFinder.h"
// #include "roboteam_tactics/utils/SkillFactory.h"

namespace rtt {
    
KeeperBlock::KeeperBlock(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard) : Skill(n, name, blackboard) {
    assert_valid<KeeperBlock>(name);
    target = -1;
    cover_dist = .4;
    reevaluate_target();
}

KeeperBlock::~KeeperBlock() {
    if (impl) impl.reset();
}

boost::optional<roboteam_msgs::WorldRobot> KeeperBlock::select_target() const {
    return most_dangerous_bot(target);
}

void KeeperBlock::reevaluate_target() {
    boost::optional<roboteam_msgs::WorldRobot> danger = select_target();
    if (!danger || target == danger->id) return;
    
    // ROS_INFO("Target: %d @ %f (can_see=%d) (preferred=%d)", danger->id, danger_score(*danger), can_see_our_goal(*danger), danger->id == target);
    
    target = danger->id;
    auto goal = (we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT)[1]; // center
    // impl = build_skill<Block>(n, "Block", "", 
        // "ROBOT_ID=%d TGT_ID=%d int:BLOCK_ID=%d block_x=%f block_y=%f block_type=%s block_arg=%f bool:invert_direction=%s",
        // GetInt("ROBOT_ID"), target, -1, goal.x, goal.y, "CIRCLE", cover_dist, "false");

    auto bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
    bb->SetInt("TGT_ID", target);
    bb->SetInt("BLOCK_ID", -1);
    bb->SetDouble("block_x", goal.x);
    bb->SetDouble("block_y", goal.y);
    bb->SetString("block_type", "CIRCLE");
    bb->SetDouble("block_arg", cover_dist);
    bb->SetBool("invert_direction", false);

    impl = std::make_shared<Block>(n, "", bb);
}

bt::Node::Status KeeperBlock::Update() {
    reevaluate_target();
    if (!impl) return bt::Node::Status::Running;
    bt::Node::Status stat = impl->Update();
    
    // Keeping is never done, unless something failed elsewhere.
    return stat == bt::Node::Status::Invalid ? stat : bt::Node::Status::Running;
}
    
}
