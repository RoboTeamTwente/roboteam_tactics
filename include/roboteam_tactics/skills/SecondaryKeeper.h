#pragma once

#include "roboteam_tactics/skills/KeeperBlock.h"

namespace rtt{

/**
 * \class SecondaryKeeper
 * \brief See YAML
 */
/*
 * Descr: Same as KeeperBlock, but stays farther away from the goal
 * Params: Same as KeeperBlock
 */
class SecondaryKeeper : public KeeperBlock {
    public:
    SecondaryKeeper(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    ~SecondaryKeeper() {}
    
    boost::optional<roboteam_msgs::WorldRobot> select_target() const override;
    
    std::string node_name() { return "SecondaryKeeper"; }
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
};
    
}
