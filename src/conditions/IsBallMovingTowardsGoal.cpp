#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallMovingTowardsGoal.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(IsBallMovingTowardsGoal);

IsBallMovingTowardsGoal::IsBallMovingTowardsGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsBallMovingTowardsGoal::Update() {
	// ROS_INFO("called IsBallMovingTowardsRobot");
	roboteam_msgs::World world = LastWorld::get();
    Vector2 theirGoalPos = LastWorld::get_their_goal_center();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

	Vector2 ballVel(world.ball.vel);
	Vector2 ballPos(world.ball.pos);
	if (ballVel.length() <= 0.5) {
    	return Status::Failure; 
    }

    Vector2 ballTrajectory = ballVel.scale(10.0 / ballVel.length());
    Vector2 closestPoint = ballTrajectory.closestPointOnVector(ballPos, theirGoalPos);

    double acceptableDeviation = field.goal_width / 2;

    if ((closestPoint - theirGoalPos).length() <= acceptableDeviation) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

}
