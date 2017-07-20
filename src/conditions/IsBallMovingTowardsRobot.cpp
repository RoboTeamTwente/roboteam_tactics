#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallMovingTowardsRobot.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(IsBallMovingTowardsRobot);

IsBallMovingTowardsRobot::IsBallMovingTowardsRobot(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsBallMovingTowardsRobot::Update() {
	// ROS_INFO("called IsBallMovingTowardsRobot");
	roboteam_msgs::World world = LastWorld::get();
	Vector2 ballVel(world.ball.vel);
	Vector2 ballPos(world.ball.pos);
	if (ballVel.length() <= 0.5) {
    	return Status::Failure;
    }

	int robotID = GetInt("ROBOT_ID");
	roboteam_msgs::WorldRobot robot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("IsBallMovingTowardsRobot could not find robot");
        return Status::Failure;
    }

    Vector2 robotPos(robot.pos);
    Vector2 ballVelUnit = ballVel.scale(1.0 / ballVel.length());
    Vector2 posDiff = robotPos - ballPos;
    if (posDiff.length() <= 0.05) {
    	return Status::Failure;
    }
    Vector2 posDiffUnit = posDiff.scale(1.0 / posDiff.length());
    double ballDir = ballVelUnit.dot(posDiffUnit);
    ROS_INFO_STREAM("ballDir: " << ballDir);

    if (ballDir >= 0.5) {
    	return Status::Success;
    } else {
    	return Status::Failure;
    }
}

}
