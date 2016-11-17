#include "roboteam_tactics/skills/BackUp.h"
#include "roboteam_tactics/utils/utils.h"
#include <cmath>

#define BACK_UP_DIST .3

namespace rtt {
    
BackUp::BackUp(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr bb)
        : Skill(n, name, bb) {
    assert_valid<BackUp>(name);
    auto bot = *(lookup_bot(blackboard->GetInt("ROBOT_ID"), true));
    original_pos = roboteam_utils::Position(bot.pos.x, bot.pos.y, bot.angle);
    double theta = fmodl(original_pos.getRot() + M_PI, 2*M_PI);
    double dx = sinl(theta) * BACK_UP_DIST;
    double dy = cosl(theta) * BACK_UP_DIST;
    target_pos = roboteam_utils::Position(original_pos.x - dx, original_pos.y - dy, original_pos.rot);
}
        
bt::Node::Status BackUp::Update() {
    auto bot = *(lookup_bot(blackboard->GetInt("ROBOT_ID"), true));
    roboteam_utils::Position current_pos = roboteam_utils::Position(bot.pos.x, bot.pos.y, bot.angle);
    
    return bt::Node::Status::Invalid;
}
    
}