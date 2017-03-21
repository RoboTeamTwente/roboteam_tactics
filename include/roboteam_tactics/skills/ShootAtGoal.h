#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class ShootAtGoal
 * \brief See YAML
 */
/*
 * Descr: Orients towards the goal, and then kicks the ball
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - kickVel:
 *       Type: Double
 *       Descr: The velocity with which the ball should be kicked
 */
class ShootAtGoal : public Skill {
public:
	ShootAtGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Vector2 DetermineTarget();
	Status Update();

private:
    roboteam_msgs::WorldRobot me;
	int robotID;
    Vector2 myPos;

    bt::Blackboard::Ptr rotateBB;
    bt::Blackboard::Ptr kickBB;
    RotateAroundPoint rotateAroundPoint;
    Kick kick;

    Status resultRotate;

    bool initializedKick;

    Draw drawer;
};


} // rtt
