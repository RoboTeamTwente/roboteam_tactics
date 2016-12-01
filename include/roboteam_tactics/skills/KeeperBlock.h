#pragma once

#include "roboteam_tactics/skills/Block.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/World.h"
#include "ros/ros.h"

namespace rtt {
    
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


    std::string node_name() { return "KeeperBlock"; }
    protected:
    std::shared_ptr<Block> impl;
    unsigned int target;
    double cover_dist;
    virtual boost::optional<roboteam_msgs::WorldRobot> select_target() const;
    
    private:
    void reevaluate_target();
};
    
}
