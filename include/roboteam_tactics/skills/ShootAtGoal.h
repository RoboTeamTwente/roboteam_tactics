#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

class ShootAtGoal : public Skill {
public:
	ShootAtGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    roboteam_utils::Vector2 DetermineTarget();
	Status Update();

private:
    roboteam_msgs::WorldRobot me;
	int robotID;
    roboteam_utils::Vector2 myPos;

    bt::Blackboard::Ptr rotateBB;
    bt::Blackboard::Ptr kickBB;
    RotateAroundPoint rotateAroundPoint;
    Kick kick;

    Status resultRotate;

    bool initializedKick;

    Draw drawer;
};


} // rtt
