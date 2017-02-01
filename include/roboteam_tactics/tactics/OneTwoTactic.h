#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/Parts.h"

namespace rtt {

class OneTwoTactic : public Tactic {
    public:
    OneTwoTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize() override;
    Status Update() override;
    
    // TODO: This should be done by the tactic automatically or not done at all!
    void ShutdownRoles();
    
    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;
    roboteam_msgs::RoleDirective scorer, assistant;

    bool canRun = true;
} ;

}
