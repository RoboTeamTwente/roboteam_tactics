#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"

namespace rtt {

/**
 * \class Dribble
 * \brief See YAML
 */
/*
 * Descr: Goes to a position while keeping the ball
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - xGoal:
 *       Type: Double
 *       Descr: The x-coordinate of the goal position
 *   - yGoal:
 *       Type: Double
 *       Descr: The y-coordinate of the goal position
 */
class Dribble : public Skill {
public:
	Dribble(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
	void stoprobot(int RobotID);
	double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);

	double cleanAngle(double angle);
    Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);
    Vector2 saveDribbleDeceleration(Vector2 reqspeed);
    
    std::string node_name() { return "Dribble"; }
private:
	bool goal1reached=false;
	ros::NodeHandle n;
	ros::Publisher pubDebugpoints;
	int robotID;
	Vector2 robotvtogoal=Vector2(0.0,0.0);
	RotateAroundPoint rotateAroundPoint;
	Vector2 prevspeed=Vector2(0.0,0.0);
    roboteam_msgs::World::_header_type::_seq_type prevSeq;
};


} // rtt
