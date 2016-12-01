#include "roboteam_tactics/skills/SecondaryKeeper.h"
#include "roboteam_tactics/utils/DangerFinder.h"

namespace rtt {
    
SecondaryKeeper::SecondaryKeeper(std::string name, 
        bt::Blackboard::Ptr blackboard) : KeeperBlock(name, blackboard) {
    cover_dist = .75;
}

boost::optional<roboteam_msgs::WorldRobot> SecondaryKeeper::select_target() const {
    return second_most_dangerous_bot();
}
    
}
