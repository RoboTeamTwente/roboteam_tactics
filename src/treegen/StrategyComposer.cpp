#include "roboteam_tactics/treegen/StrategyComposer.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>

namespace rtt {
    
bool StrategyComposer::initialized = false;    
bt::BehaviorTree StrategyComposer::mainStrategy;

/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Any UNSET ref states will fall back to the NORMAL_PLAY strategy.
 * If the NORMAL_PLAY strategy is UNSET, bad things will happen.
 */
const std::map<RefState, const char*> StrategyComposer::MAPPING = {
        { RefState::HALT,                 UNSET },
        { RefState::STOP,                 UNSET },
        { RefState::NORMAL_START,         UNSET },
        { RefState::FORCED_START,         UNSET },
        { RefState::PREPARE_KICKOFF_US,   UNSET },
        { RefState::PREPARE_KICKOFF_THEM, UNSET },
        { RefState::PREPARE_PENALTY_US,   UNSET },
        { RefState::PREPARE_PENALTY_THEM, UNSET },
        { RefState::DIRECT_FREE_US,       UNSET },
        { RefState::DIRECT_FREE_THEM,     "FreeKickDefenceStrategy"},
        { RefState::INDIRECT_FREE_US,     UNSET },
        { RefState::INDIRECT_FREE_THEM,   "FreeKickDefenceStrategy"},
        { RefState::TIMEOUT_US,           UNSET },
        { RefState::TIMEOUT_THEM,         UNSET },
        { RefState::GOAL_US,              UNSET },
        { RefState::GOAL_THEM,            UNSET },
        { RefState::BALL_PLACEMENT_US,    UNSET },
        { RefState::BALL_PLACEMENT_THEM,  UNSET },
        { RefState::NORMAL_PLAY,          "DemoStrategy"}
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
    
    const char* defName = MAPPING.at(RefState::NORMAL_PLAY);
    bt::BehaviorTree def = repo.at(std::string(defName));
    
    for (unsigned int i = 0; i < MAPPING.size() - 1; i++) {
        const char* strat = MAPPING.at(static_cast<RefState>(i));
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
