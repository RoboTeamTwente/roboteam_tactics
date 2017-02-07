#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

// Forward declare instead of including
class Block;
    
class KeeperBlock : public Skill {
    public:
    KeeperBlock(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~KeeperBlock();
    
    Status Update() override final;
    
    static VerificationMap required_params() {
        VerificationMap params;
        // params["keeper"] = BBArgumentType::Int;
        return params;
    }


    std::string node_name() override { return "KeeperBlock"; }
    protected:
    std::shared_ptr<Block> impl;
    unsigned int target;
    double cover_dist;
    virtual boost::optional<roboteam_msgs::WorldRobot> select_target() const;
    
    private:
    void reevaluate_target();
};
    
}
