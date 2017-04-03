#pragma once

#include "roboteam_tactics/Parts.h"
#include "unique_id/unique_id.h"
#include "roboteam_msgs/RoleDirective.h"

namespace rtt {
    
class WanderTactic : public Tactic {
    public:
    WanderTactic(std::string name = "", bt::Blackboard::Ptr bb = nullptr);
    void Initialize();
    Status Update();

    private:
    
    static constexpr int DEFAULT_NEAR_BALL = 1,
                         DEFAULT_NEAR_GOAL = 1,
                         DEFAULT_MOST_IMPORTANT_QUADRANT = 2,
                         DEFAULT_SECONDARY_QUARDRANT = 1;
    
    std::vector<boost::uuids::uuid> tokens;
    bool valid;

    int pickMostImportantQuad() const;
};
    
}