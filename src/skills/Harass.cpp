#include "roboteam_tactics/skills/Harass.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {
    
bt::Blackboard::Ptr make_bb(const bt::Blackboard::Ptr base, bool invert) {
    bt::Blackboard::Ptr subBB = std::make_shared<bt::Blackboard>();
    merge_blackboards(subBB, base);
    subBB->SetInt("BLOCK_ID", BLOCK_BALL_ID);
    subBB->SetString("block_type", "COVER");
    subBB->SetBool("invert_direction", invert);
    return subBB;
}
    
Harass::Harass(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {
    assert_valid<Harass>(name);
    {
        bt::Blackboard::Ptr subBB = make_bb(bb, false);
        block_kick = std::make_unique<Block>((name == "" ? "" : name + "_") + "Block_Kick", subBB);
    }
    {
        bt::Blackboard::Ptr subBB = make_bb(bb, true);
        block_get = std::make_unique<Block>((name == "" ? "" : name + "_") + "Block_Get", subBB);
    }
    target = bb->GetInt("TGT_ID");
}

bt::Node::Status Harass::Update() {
    if (bot_has_ball(target, false, LastWorld::get().ball)) {
        return block_kick->Update();
    }    
    return block_get->Update();
}
    
}