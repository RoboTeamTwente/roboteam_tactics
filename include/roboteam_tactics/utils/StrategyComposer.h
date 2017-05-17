#pragma once

#include <boost/optional.hpp>
#include <map>

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_utils/LastRef.h"

namespace rtt {

/**
 * \brief The value which indicates that a referee state has no special treatment in the strategy
 */
extern const std::string UNSET;
    
/**
 * \class StrategyComposer
 * \brief Builds a single, static, horribly ugly Strategy tree by combining ones meant for different referee states.
 */
class StrategyComposer {
private:
    StrategyComposer() = delete;
    static std::shared_ptr<bt::BehaviorTree> mainStrategy;
    static void init();
    static bool initialized;
    
    static constexpr unsigned int CASE_COUNT = 19;
    
    // SET THIS IN StrategyComposer.cpp !!
    static const std::map<RefState, boost::optional<std::string>> MAPPING;
    
    class Forwarder final : public bt::Leaf {
        public:
        Forwarder(bt::Blackboard::Ptr bb, bt::Node::Ptr target);
        bt::Node::Status Update() override;
        void Initialize() override;
        void Terminate(bt::Node::Status status) override;
        private:
        bt::Node::Ptr target;
    };
public:
    static std::shared_ptr<bt::BehaviorTree> getMainStrategy();
};
    
}
