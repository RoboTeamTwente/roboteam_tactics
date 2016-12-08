#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class GoToSideTactic : public Tactic {
    public:
    GoToSideTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "GoToSideTactic"; }
    private:
    std::vector<boost::uuids::uuid> tokens;
} ;

}
