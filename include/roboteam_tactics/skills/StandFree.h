#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Draw.h"

namespace rtt {
    
class StandFree : public Skill {

public:
    StandFree(std::string name, bt::Blackboard::Ptr blackboard);  
	boost::optional<Cone> MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, Vector2 myPos, Vector2 targetPos);
    Status Update();
private:
	GoToPos goToPos;
    Draw drawer;
};
    
}