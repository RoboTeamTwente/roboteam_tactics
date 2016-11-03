#include "roboteam_tactics/conditions/CanPassSafely.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

CanPassSafely::CanPassSafely(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    
}

bt::Node::Status CanPassSafely::Update (){
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	int passToRobot = GetInt("passToRobot");
	if (myID == passToRobot) {
		ROS_INFO("you're trying to pass to yourself, you silly you");
		return Status::Invalid;
	}

	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y);	
	roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(world.us.at(passToRobot).pos.x, world.us.at(passToRobot).pos.y);
	roboteam_utils::Vector2 ballTrajectory = targetPos - myPos;

	bool failure;
	for (size_t i = 0; i < world.them.size(); i++) {
		roboteam_utils::Vector2 theirPos = roboteam_utils::Vector2(world.them.at(i).pos.x, world.them.at(i).pos.y);
		roboteam_utils::Vector2 vectorToOtherRobot = theirPos - myPos;
		double angle = ballTrajectory.angle() - vectorToOtherRobot.angle();
		double projectionLength = vectorToOtherRobot.length() * cos(angle);

		// projectionOnBallTrajectory contains the point on the ball trajectory to which the opponent is closest
		roboteam_utils::Vector2 projectionOnBallTrajectory;
		if (projectionLength > ballTrajectory.length()) {
			projectionOnBallTrajectory = targetPos;
		} else if (projectionLength < 0) {
			projectionOnBallTrajectory = myPos;
		} else {
			projectionOnBallTrajectory = ballTrajectory.scale(projectionLength / ballTrajectory.length()) + myPos;
		}

		// Estimate how long it will take for the ball to get there
		double ballSpeed = 4.0;
		double ballMass = 0.043;
		double ballFrictionCoefficient = 0.05;
		double timeStep = 0.01;
		double time = 0.0;
		double travelledDistance = 0.0;
		while (travelledDistance < projectionLength) {
			double frictionForce = ballMass*9.81 * ballFrictionCoefficient;
			double ballAcc = frictionForce/ballMass;
			ballSpeed -= ballAcc*timeStep;
			// if (i == 0) {
			// 	ROS_INFO_STREAM("ballSpeed: " << ballSpeed);
			// }
			if (ballSpeed < 0) {
				ballSpeed = 0;
				break;
			}
			travelledDistance += ballSpeed*timeStep;
			time += timeStep;
		}
		// ROS_INFO_STREAM(time << " seconds until ball travels distance " << projectionLength);

		auto bb2 = std::make_shared<bt::Blackboard>();
		bb2->SetInt("ROBOT_ID", i);
		bb2->SetString("whichTeam", "them");
		bb2->SetDouble("xGoal", projectionOnBallTrajectory.x);
		bb2->SetDouble("yGoal", projectionOnBallTrajectory.y);
		bb2->SetDouble("timeLimit", time);
		CanReachPoint canReachPoint("", bb2);

		if (canReachPoint.Update() == Status::Success) {
			ROS_INFO_STREAM("robot " << i << " can reach this point within the time limit...");
			failure = true;
		}
	}
	if (failure) {
		return Status::Failure;
	} else {
		return Status::Success;
	}
	return Status::Invalid;
}

} //rtt