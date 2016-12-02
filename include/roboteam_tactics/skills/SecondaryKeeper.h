#pragma once

#include "roboteam_tactics/skills/KeeperBlock.h"

namespace rtt{
    
class SecondaryKeeper : public KeeperBlock {
    public:
    SecondaryKeeper(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    ~SecondaryKeeper() {}
    
    boost::optional<roboteam_msgs::WorldRobot> select_target() const override;
    
    std::string node_name() { return "SecondaryKeeper"; }
    static VerificationMap required_params() {
        VerificationMap params;
        params["keeper"] = BBArgumentType::Int;
        return params;
    }
};
    
}
