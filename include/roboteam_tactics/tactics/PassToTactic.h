#pragma once

#include "unique_id/unique_id.h"
#include "std_msgs/Int32.h"

#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class PassToTactic : public Tactic {
    public:
    PassToTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    void ShutdownRoles();
    Status Update();
    
    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;
    bool tacticSucceeded = false;

    roboteam_msgs::RoleDirective passer;
    roboteam_msgs::RoleDirective receiver;
} ;

}
