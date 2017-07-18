#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/skills/Block.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include <cassert>

namespace rtt {
    
class InterceptorsTactic : public Tactic {
    
    public:
    InterceptorsTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr)
        : Tactic(name, blackboard), failure(false) {}
        
    void Initialize() override;
    Status Update() override;
    
    private:
    std::vector<boost::uuids::uuid> tokens;
    bool failure;
    void assign(int own, const roboteam_msgs::WorldRobot& opp);
    
};
    
}
