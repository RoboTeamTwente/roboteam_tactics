#include "roboteam_tactics/treegen/StrategyComposer.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>

namespace rtt {
    
bool StrategyComposer::initialized = false;    
bt::BehaviorTree StrategyComposer::mainStrategy;

/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Please do not remove the comments.
 * Any UNSET ref states will fall back to the NORMAL_PLAY strategy.
 * If the NORMAL_PLAY strategy is UNSET, bad things will happen.
 */
constexpr std::array<const char*, StrategyComposer::CASE_COUNT> StrategyComposer::MAPPING = {
        UNSET, // HALT
        UNSET, // STOP
        UNSET, // NORMAL_START
        UNSET, // FORCE_START
        UNSET, // PREPARE_KICKOFF_US
        UNSET, // PREPARE_KICKOFF_THEM
        UNSET, // PREPARE_PENALTY_US
        UNSET, // PREPARE_PENALTY_THEM
        UNSET, // DIRECT_FREE_US
        "FreeKickDefenceStrategy", // DIRECT_FREE_THEM
        UNSET, // INDIRECT_FREE_US
        "FreeKickDefenceStrategy", // INDIRECT_FREE_THEM
        UNSET, // TIMEOUT_US
        UNSET, // TIMEOUT_THEM
        UNSET, // GOAL_US
        UNSET, // GOAL_THEM
        UNSET, // BALL_PLACEMENT_US
        UNSET, // BALL_PLACEMENT_THEM
        "DemoStrategy" // NORMAL_PLAY
};

bt::BehaviorTree StrategyComposer::getMainStrategy() {
    if (!initialized) init();
    return mainStrategy;
}



void StrategyComposer::init() {
    if (initialized) return;
    
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>(bt::Blackboard());
    std::shared_ptr<RefStateSwitch> rss;
    
    auto repo = factories::getRepo<bt::BehaviorTree>();
    
    constexpr const char* defName = MAPPING[CASE_COUNT - 1];
    bt::BehaviorTree def = repo.at(std::string(defName));
    
    for (unsigned int i = 0; i < MAPPING.size() - 1; i++) {
        const char* strat = MAPPING[i];
        if (strat == UNSET) {
            Forwarder fw(bb, def);
            rss->AddChild(std::make_shared<Forwarder>(fw));
        } else {
            bt::BehaviorTree node = repo.at(std::string(strat));
            rss->AddChild(std::make_shared<bt::BehaviorTree>(node));
        }
    }
    
    mainStrategy = bt::BehaviorTree();
    mainStrategy.SetRoot(rss);
}    
    
StrategyComposer::Forwarder::Forwarder(bt::Blackboard::Ptr bb, bt::Node& target) : bt::Leaf(bb), target(target) {}
bt::Node::Status StrategyComposer::Forwarder::Update() { return target.Update(); }
void StrategyComposer::Forwarder::Initialize() { target.Initialize(); }
void StrategyComposer::Forwarder::Terminate(bt::Node::Status status) { target.Terminate(status); }    
    
}
