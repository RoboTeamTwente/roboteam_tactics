#include <string>

#include "roboteam_tactics/bt/RefStateSwitch.h"
#include "roboteam_tactics/treegen/StrategyComposer.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {
    
bool StrategyComposer::initialized = false;    
const std::string UNSET = "<<UNSET>>";
std::shared_ptr<bt::BehaviorTree> StrategyComposer::mainStrategy;

/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Any UNSET ref states will fall back to the NORMAL_PLAY strategy.
 * If the NORMAL_PLAY strategy is UNSET, bad things will happen.
 */
const std::map<RefState, std::string> StrategyComposer::MAPPING = {
        { RefState::HALT,                 "rtt_dennis/HaltStrategy" },
        { RefState::STOP,                 "rtt_dennis/HaltStrategy" },
        { RefState::NORMAL_START,         UNSET },
        { RefState::FORCED_START,         UNSET },
        { RefState::PREPARE_KICKOFF_US,   "rtt_bob/prepare_kickoff_us"},
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
        // qualification/StandByStrat
        // rtt_bob/NormalStrategy
        { RefState::NORMAL_PLAY,          "rtt_bob/NormalStrategy"}
};

std::shared_ptr<bt::BehaviorTree> StrategyComposer::getMainStrategy() {
    if (!initialized) init();
    return mainStrategy;
}

void StrategyComposer::init() {
    // Return if not initialized
    if (initialized) return;
    
    // Construct the global bb and the refstate switch
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>(bt::Blackboard());
    std::shared_ptr<RefStateSwitch> rss = std::make_shared<RefStateSwitch>();
    
    // Get the factory that stores all the behavior trees
    namespace f = ::rtt::factories;
    auto const & repo = f::getRepo<f::Factory<bt::BehaviorTree>>();

    // Get the default tree factory map entry
    std::string defName = MAPPING.at(RefState::NORMAL_PLAY);
    auto defIt = repo.find(defName);

    // If it exists, set it as default
    bt::BehaviorTree::Ptr def;
    if (defIt != repo.end()) {
        def = defIt->second("", bb);
    } else {
        std::cerr << "Could not find a tree for default strategy tree \""
                  << defName
                  << "\". Possibly \"refresh_b3_projects.sh\" needs to be run "
                  << "or a non-existent tree was selected.\n";
        return;
    }
    
    // For each mapping...
    for (unsigned int i = 0; i < MAPPING.size(); i++) {
        // Get the name of the desired strategy
        std::string strat = MAPPING.at(static_cast<RefState>(i));

        // If it's unset use the default
        if (strat == UNSET) {
            Forwarder fw(bb, def);
            rss->AddChild(std::make_shared<Forwarder>(fw));
        } else {
            // Else try to find the desired strategy tree
            auto stratIt = repo.find(strat);

            if (stratIt != repo.end()) {
                // If so, set it
                auto node = repo.at(strat)("", bb);
                rss->AddChild(node);
            } else {
                // Else it's not there, abort!
                std::cerr << "Could not find a tree for \""
                          << strat
                          << "\". Possibly \"refresh_b3_projects.sh\" needs to be run "
                          << "or a non-existent tree was selected.\n";
                return;
            }
        }
    }
    
    // The main strategy is now properly initialized
    mainStrategy = std::make_shared<bt::BehaviorTree>();
    mainStrategy->SetRoot(rss);
    initialized = true;
}    
    
StrategyComposer::Forwarder::Forwarder(bt::Blackboard::Ptr bb, bt::Node::Ptr target) : bt::Leaf(bb), target(target) {}
bt::Node::Status StrategyComposer::Forwarder::Update() { return target->Update(); }
void StrategyComposer::Forwarder::Initialize() { target->Initialize(); }
void StrategyComposer::Forwarder::Terminate(bt::Node::Status status) { target->Terminate(status); }    
    
}
