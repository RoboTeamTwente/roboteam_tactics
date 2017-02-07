#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include <cassert>

namespace rtt {

class SecondaryKeeperTactic : public SingleBotTactic {
    public:
    SecondaryKeeperTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr)
        : SingleBotTactic(name, blackboard) {}
        
    void Initialize() override;
    Status Update() override;
    
};
   
}