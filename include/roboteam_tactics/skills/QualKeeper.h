#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/GoToPos.h"

namespace rtt {

/**
 * \class QualKeeper
 * \brief See YAML
 */
/*
 * Descr: Simple keeper for qualification
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 */
class QualKeeper : public Skill {
public:
	QualKeeper(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);;
	Status Update();

private:
    GoToPos goToPos;
};


} // rtt
