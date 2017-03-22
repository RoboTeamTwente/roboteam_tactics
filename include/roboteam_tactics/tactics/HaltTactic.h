#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class HaltTactic final : public Tactic {
public:
    HaltTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    
private:
    std::vector<boost::uuids::uuid> tokens;
    void sendStops();
};
    
}
