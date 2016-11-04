#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include <boost/optional.hpp>

namespace rtt {
    
class IHaveBall : public Condition {


public:
    IHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);  
    Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        params["our_team"] = BBArgumentType::Bool;
        return params;
    }
    
    std::vector<roboteam_msgs::World> success_states() const override;
    std::vector<roboteam_msgs::World> fail_states() const override;
private:
    int me;
    bool us;
    boost::optional<roboteam_msgs::WorldRobot> find_bot_pos(const roboteam_msgs::World&) const;
};
    
}