#pragma once

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/DangerFinder.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "unique_id/unique_id.h"
#include <cassert>

namespace rtt {
    
class FreeKickDefenceTactic : public Tactic {
    public:
    FreeKickDefenceTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    ros::NodeHandle n;

    bool isInitialized;

    private:
    std::vector<boost::uuids::uuid> tokens;

    time_point start;
    
};
    
}