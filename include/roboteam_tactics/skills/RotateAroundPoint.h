#pragma once
#include <cstdint>

#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class RotateAroundPoint
 * \brief See YAML
 */
/*
 * Descr: Rotates around a certain point
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - center:
 *       Type: String
 *       Can be:
 *         - ball: Rotate around the ball
 *         - point: Rotate around a given point
 *       Descr: Determines what to rotate around
 *   - faceTowardsPosx:
 *       Type: Double
 *       Descr: The x-coordinate of the point to face towards
 *   - faceTowardsPosy:
 *       Type: Double
 *       Descr: The y-coordinate of the point to face towards
 *   - w:
 *       Type: Double
 *       Descr: The angular velocity to maintain
 *   - centerx:
 *       Type: Double
 *       Used when: center == point
 *       Descr: The x-coordinate of the point to rotate around
 *   - centery:
 *       Type: Double
 *       Used when: center == point
 *       Descr: The y-coordinate of the point to rotate around
 *   - radius:
 *       Type: Double
 *       Used when: center == point
 *       Descr: The distance to keep from the rotation center
 *   - maxv:
 *       Type: Double
 *       Descr: Indicates maximum speed. Default: 1 m/s
 *   - quiet:
 *       Type: Bool
 *       Descr: Stops the skill from sending stop commands
 */
class RotateAroundPoint : public Skill {
public:
	
    RotateAroundPoint(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();   
    Status checkAndSetArguments();
    void stoprobot(int RobotID);
    std::string node_name() { return "RotateAroundPoint"; }

private:
	bt::Blackboard::Ptr goto_bb;
	uint32_t prevworldseq;
	bool firstworld=true;
	int robotID;
	double targetAngle;
	roboteam_msgs::WorldRobot robot;
	roboteam_msgs::WorldBall ball;
	Vector2 faceTowardsPos;
	double rotw;
	Vector2 center;
   	double radius;
   	GoToPos goToPos;
   	double rotPconstant;
	double radiusPconstant;
	double turnPconstant;
} ;

} // rtt
