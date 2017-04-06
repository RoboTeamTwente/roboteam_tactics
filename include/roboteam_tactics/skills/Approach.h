#pragma once

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"

namespace rtt {

class Approach : public Skill {
    public:
    Approach(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
    private:
    static constexpr double DEFAULT_DISTANCE = .3;
    std::unique_ptr<GoToPos> gtp;
};
    
}