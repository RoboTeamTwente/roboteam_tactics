	#include <map>
#include <boost/optional.hpp>
#include <string>

#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_tactics/utils/StrategyComposer.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace b = boost;

namespace rtt {
    
bool StrategyComposer::initialized = false;    
// const std::string UNSET = "<<UNSET>>";
std::shared_ptr<bt::BehaviorTree> StrategyComposer::mainStrategy;

using namespace std::string_literals;

/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Any UNSET ref states will fall back to the NORMAL_PLAY strategy.
 * If the NORMAL_PLAY strategy is UNSET, bad things will happen.
 *
 * ////////////////////////////////
 * // Don't forget the s suffix! //
 * ////////////////////////////////
 *
 */
const std::map<RefState, b::optional<std::string>> StrategyComposer::MAPPING = {
        ///////////////////////////////////////////////////////////////////////
        // Explicitly unused states that should redirect towards normal play //
        ///////////////////////////////////////////////////////////////////////
        
        { RefState::NORMAL_START          , "rtt_jim/SimpleAttStrat"s      } ,
        { RefState::FORCED_START          , b::none                        } ,
        
        ////////////////////////////////////////////////////
        // Ref states that have a specific implementation //
        ////////////////////////////////////////////////////
        
        { RefState::HALT                  , "rtt_dennis/HaltStrategy"s           } ,
        { RefState::STOP                  , "rtt_dennis/HaltStrategy"s           } ,
        { RefState::PREPARE_KICKOFF_US    , "rtt_dennis/StopStrategy"s           } ,
        { RefState::PREPARE_KICKOFF_THEM  , "rtt_dennis/StopStrategy"s           } ,
        { RefState::PREPARE_PENALTY_US    , "rtt_dennis/StopStrategy"s           } ,
        { RefState::PREPARE_PENALTY_THEM  , "rtt_dennis/StopStrategy"s           } ,

        // rtt_ewoud/FreeKickTakeStrategy
        { RefState::DIRECT_FREE_US        , "rtt_bob/W5_DirectFreeUs"s           } ,

        // FreeKickDefenceStrategy
        { RefState::DIRECT_FREE_THEM      , "rtt_bob/W5_DirectFreeThem"s         } ,

        // rtt_ewoud/FreeKickTakeStrategy
        { RefState::INDIRECT_FREE_US      , "rtt_bob/W5_IndirectFreeUs"s         } ,

        // FreeKickDefenceStrategy
        { RefState::INDIRECT_FREE_THEM    , "rtt_bob/W5_IndirectFreeThem"s       } ,
        { RefState::TIMEOUT_US            , "rtt_jim/TimeOutStrat"s              } ,
        { RefState::TIMEOUT_THEM          , "rtt_dennis/StopStrategy"s           } ,
        { RefState::GOAL_US               , "rtt_dennis/StopStrategy"s           } ,
        { RefState::GOAL_THEM             , "rtt_dennis/StopStrategy"s           } ,
        { RefState::BALL_PLACEMENT_US     , "rtt_bob/BallPlacementUsStrategy"s   } ,
        { RefState::BALL_PLACEMENT_THEM   , "rtt_jim/TimeOutStrat"s              } ,

        //////////////////////////
        // Our custom refstates //
        //////////////////////////
        
        // qualification/StandByStrategy
        // rtt_bob/NormalStrategy
        { RefState::DO_KICKOFF            , "rtt_bob/KickoffWithRunStrategy"s    } ,
        { RefState::DEFEND_KICKOFF        , "rtt_dennis/KickoffDefenseStrategy"s } ,
        { RefState::DO_PENALTY            , "rtt_bob/W5_DoPenalty"s              } ,
        { RefState::DEFEND_PENALTY        , "rtt_bob/W5_DefendPenalty"s          } ,

        { RefState::NORMAL_PLAY           , "rtt_jim/SimpleAttStrat"s            } ,
} ;

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
    std::string defName;
    auto defNameIt = MAPPING.find(RefState::NORMAL_START);
    if (defNameIt != MAPPING.end() && defNameIt->second) {
        defName = *defNameIt->second;
    } else {
        std::cerr << "Could not find a normal play strategy! "
                  << "Please verify that the static MAPPING variable contains a normal play.\n"
                  ;
        return;
    }

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
    
    for (auto refState : ALL_REFSTATES) {
        // Get the name of the desired strategy
        auto stratNameOpt = MAPPING.at(refState);

        // If it's unset use the default
        if (!stratNameOpt) {
            std::cout << "Forwarding tree: " << refStateToString(refState) << "\n";
            Forwarder fw(bb, def);
            // rss->AddChild(std::make_shared<Forwarder>(fw));
            rss->AddStrategy(refState, std::make_shared<Forwarder>(fw));
        } else {
            auto stratName = *stratNameOpt;
            // Else try to find the desired strategy tree
            auto stratIt = repo.find(stratName);

            if (stratIt != repo.end()) {
                // If so, set it
                auto node = stratIt->second("", bb);
                rss->AddStrategy(refState, node);
            } else {
                // Else it's not there, abort!
                std::cerr << "Could not find a tree for \""
                          << stratName
                          << "\" for refstate: \""
                          << refStateToString(refState)
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
