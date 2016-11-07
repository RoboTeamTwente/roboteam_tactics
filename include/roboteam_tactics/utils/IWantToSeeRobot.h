#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_utils/Vector2.h"
#include <boost/optional.hpp>

namespace rtt {
    
class IWantToSeeRobot : public Condition {

public:
    IWantToSeeRobot(std::string name, bt::Blackboard::Ptr blackboard);  
    Status Update();
private:
};
    
}