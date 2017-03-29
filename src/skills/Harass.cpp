#include "roboteam_tactics/skills/Harass.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG Harass

namespace rtt {
    
RTT_REGISTER_SKILL(Harass);
    
bt::Blackboard::Ptr make_bb(const bt::Blackboard::Ptr base, int blockId, const std::string& name, bool invert) {
    bt::Blackboard::Ptr subBB = std::make_shared<bt::Blackboard>();
    merge_blackboards(subBB, base);
    subBB->SetInt(name + "_BLOCK_ID", blockId);
    subBB->SetString(name + "_block_type", "COVER");
    subBB->SetBool(name + "_invert_direction", invert);
    return subBB;
}
    
Harass::Harass(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {
    assert_valid<Harass>(name);
    target = GetInt("TGT_ID");
    {
        std::string nodeName = (name == "" ? "" : name + "_") + "Block_Kick";
        bt::Blackboard::Ptr subBB = make_bb(bb, 1, nodeName, true);
        subBB->SetInt(nodeName + "_TGT_ID", target);
        block_kick = std::make_unique<Block>(nodeName, subBB);
    }
    {
        std::string nodeName = (name == "" ? "" : name + "_") + "Block_Get";
        bt::Blackboard::Ptr subBB = make_bb(bb, BLOCK_BALL_ID, nodeName, false);
        subBB->SetInt(nodeName + "_TGT_ID", target);
        block_get = std::make_unique<Block>(nodeName, subBB);
    }
}

bt::Node::Status Harass::Update() {
    auto world = LastWorld::get();
    auto ball = world.ball;
    Vector2 ballPos(ball.pos);
    Vector2 myPos(lookup_bot(blackboard->GetInt("ROBOT_ID"), true, &world)->pos);
    if (bot_has_ball(target, false, LastWorld::get().ball)) {
        bt::Node::Status s = block_kick->Update();
        if (s == bt::Node::Status::Failure) {
            ROS_ERROR("Failure");
            return bt::Node::Status::Failure;
        }
        return s;
    }
    if (ballPos.dist2(myPos) <= .2) {
        if (!get_ball) {
            std::string nodeName = (name == "" ? "" : name + "_") + "Get_Ball";
            get_ball = std::make_unique<GetBall>(nodeName, blackboard);
        }
        return get_ball->Update();
    }
    bt::Node::Status s = block_get->Update();
    if (s == bt::Node::Status::Failure) {
        ROS_ERROR("Failure");
        return bt::Node::Status::Failure;
    }
    return s;
}
    
}