#include "roboteam_msgs/World.h"

#include "roboteam_tactics/conditions/CanPassSafely.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(CanPassSafely);

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
	ROS_INFO_STREAM("condition CanPassSafely, from " << myID << " to " << passToRobot);

	Vector2 myPos = Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y);	
	Vector2 targetPos = Vector2(world.us.at(passToRobot).pos.x, world.us.at(passToRobot).pos.y);
	Vector2 ballTrajectory = targetPos - myPos;

	bool failure;
	for (size_t i = 0; i < world.them.size(); i++) {
		Vector2 theirPos = Vector2(world.them.at(i).pos.x, world.them.at(i).pos.y);
		
		// projectionOnBallTrajectory contains the point on the ball trajectory to which the opponent is closest
		Vector2 projectionOnBallTrajectory = ballTrajectory.closestPointOnVector(myPos, theirPos);

		// Estimate how long it will take for the ball to get there
		double travelDistanceToClosestPoint = (projectionOnBallTrajectory-myPos).length();
		double ballSpeed = 4.0;
		double ballMass = 0.043;
		double ballFrictionCoefficient = 0.05;
		double timeStep = 0.01;
		double time = 0.0;
		double travelledDistance = 0.0;
		while (travelledDistance < travelDistanceToClosestPoint) {
			double frictionForce = ballMass*9.81 * ballFrictionCoefficient;
			double ballAcc = frictionForce/ballMass;
			ballSpeed -= ballAcc*timeStep;
			if (ballSpeed < 0) {
				ballSpeed = 0;
				break;
			}
			travelledDistance += ballSpeed*timeStep;
			time += timeStep;
		}
		ROS_INFO_STREAM("calling canReachPoint with blackboard: ROBOT_ID "<<i<<", whichTeam: them, xGoal "<<projectionOnBallTrajectory.x<<", yGoal "<<projectionOnBallTrajectory.y<< ", timeLimit "<<time);
		// Check whether the opponent can reach the calculated point within the calculated time
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
