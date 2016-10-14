#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

	GetBall::GetBall(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(n, name, blackboard)
            , goToPos(n, "", private_bb) {
        pubGetBall = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
        ROS_INFO("GetBall constructor");
	}

	void GetBall::UpdateArgs(int robotIDInput) {
		robotID = robotIDInput;
	}

	bt::Node::Status GetBall::Update (){
		// ROS_INFO("update");
		roboteam_msgs::World world = LastWorld::get();
		while (world.robots_yellow.size() == 0) {
			return Status::Running;
		}
		roboteam_msgs::WorldBall ball = world.ball;
		roboteam_msgs::WorldRobot robot = world.robots_yellow.at(robotID);

		roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
		roboteam_utils::Vector2 posDiff = ballPos-robotPos;
		// Robot radius = 0.09
		roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
		roboteam_utils::Vector2 targetPos = ballPos - posDiffNorm.scale(0.09);
		double targetAngle = posDiff.angle();

		if (posDiff.length() < 0.105) {
			ROS_INFO("Target position reached, starting dribbler...");
			roboteam_msgs::RobotCommand command;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w_vel = 0.0;
			command.dribbler = true;
			pubGetBall.publish(command);
			ros::spinOnce();
			return Status::Success;
		} else {
			if (fabs(prevTargetPos.x-targetPos.x) > 0.03 || fabs(prevTargetPos.y-targetPos.y) > 0.03 || fabs(prevTargetAngle-targetAngle) > 0.03) {
                private_bb->SetDouble("xGoal", targetPos.x);
                private_bb->SetDouble("yGoal", targetPos.y);
                private_bb->SetDouble("wGoal", targetAngle);
				// ROS_INFO_STREAM(goal);
			}
			goToPos.Update();
			// oldGoal = goal;
			prevTargetPos = roboteam_utils::Vector2(targetPos.x, targetPos.y);
			prevTargetAngle = targetAngle;
			return Status::Running;
		}
	// }
};

} // rtt
