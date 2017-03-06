#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include <array>

namespace rtt {

static constexpr const char* UNSET = "<<TODO>>";
    
class StrategyComposer {
private:
    StrategyComposer() = delete;
    static bt::BehaviorTree mainStrategy;
    static void init();
    static bool initialized;
    
    static constexpr unsigned int CASE_COUNT = 19;
    
    // SET THIS IN StrategyComposer.cpp !!
    static const std::array<const char*, CASE_COUNT> MAPPING;
    
    class Forwarder final : public bt::Leaf {
        public:
        Forwarder(bt::Blackboard::Ptr bb, bt::Node& target);
        bt::Node::Status Update() override;
        void Initialize() override;
        void Terminate(bt::Node::Status status) override;
        private:
        bt::Node& target;
    };
public:
    static bt::BehaviorTree getMainStrategy();
};
    
}
