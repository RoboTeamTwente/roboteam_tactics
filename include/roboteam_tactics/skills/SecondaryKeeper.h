#pragma once

#include "roboteam_tactics/skills/KeeperBlock.h"

namespace rtt{
    
class SecondaryKeeper : public KeeperBlock {
    public:
    SecondaryKeeper(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    ~SecondaryKeeper() {}
    
    boost::optional<roboteam_msgs::WorldRobot> select_target() const override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["keeper"] = BBArgumentType::Int;
        return params;
    }
};
    
}