#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/SecondaryKeeper.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {
    
RTT_REGISTER_SKILL(SecondaryKeeper);

SecondaryKeeper::SecondaryKeeper(std::string name, 
        bt::Blackboard::Ptr blackboard) : KeeperBlock(name, blackboard) {
    cover_dist = .75;
}

boost::optional<roboteam_msgs::WorldRobot> SecondaryKeeper::select_target() const {
    return getBotFromDangerList(1);
}
    
}
