#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

/**
 * abcdsfdsalkjja
 */

class Jelle_DemoPlay : public Tactic {
public:
    Jelle_DemoPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize() override;
    Status Update() override;
    
    std::string node_name() override { return "Jelle_DemoPlay"; }

private:
    boost::uuids::uuid token;
    bool failed;
    bool succeeded;
    int Keeper_id;
    int Attacker_id;
    
    roboteam_msgs::RoleDirective rd1;
    roboteam_msgs::RoleDirective rd2;

} ;

} // rtt


