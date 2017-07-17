#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/skills/AimAtSafe.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

RTT_REGISTER_SKILL(AimAtSafe);

AimAtSafe::AimAtSafe(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , rotateAroundPoint("", private_bb) {
}

int stage=0;
Vector2 goalPos;

bt::Node::Status AimAtSafe::Update (){

	roboteam_msgs::World world = LastWorld::get();
	//bool setRosParam = GetBool("setRosParam");

	int robotID = blackboard->GetInt("ROBOT_ID");

	roboteam_msgs::WorldRobot robot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("AimAtSafe could not find robot");
        return Status::Failure;
    }

	Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	
	roboteam_msgs::WorldBall ball = world.ball;
	Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
	
	// if (robotID == 1) ROS_INFO_STREAM("AimAtSafe Update");
	std::string destination = GetString("At");

	// Check is world contains a sensible message. Otherwise wait, it might the case that AimAtSafe::Update
	// is called before the first world state update
	if (world.us.size() == 0) {
		return Status::Running;
	}

	Vector2 passTo;

	if (destination=="robot"){
        int AtRobotID = GetInt("AtRobot");
        auto possibleBot = getWorldBot(AtRobotID);
        if (possibleBot) {
            passTo = Vector2(possibleBot->pos);
        } else {
            return Status::Failure;
        }
	} else if(destination=="theirgoal"){
        passTo = LastWorld::get_their_goal_center();
	} else if(destination=="ourgoal"){
        passTo = LastWorld::get_our_goal_center();
	} else if (destination == "position") {
        passTo = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    }


    private_bb->SetInt("ROBOT_ID", robotID);
	private_bb->SetString("center", "ball");
	private_bb->SetDouble("w",3.0);

	Status result = Status::Failure;

	
    switch (stage){
    case 0: 
    	goalPos=ballPos*2-robotPos;
    	std::cout << "stage0";
    	stage=1;
    case 1: // robot moving to point away from ball
	    private_bb->SetDouble("faceTowardsPosx", goalPos.x);
	    private_bb->SetDouble("faceTowardsPosy", goalPos.y);
	    private_bb->SetDouble("radius", 0.3);
	    result = rotateAroundPoint.Update();
		if (result == Status::Success) {
			stage=2;
		}
    break;
    case 2: // moving around ball
	    private_bb->SetDouble("faceTowardsPosx", passTo.x);
	    private_bb->SetDouble("faceTowardsPosy", passTo.y);
	    private_bb->SetDouble("radius", 0.3);
	    result = rotateAroundPoint.Update();
		if (result == Status::Success) {
			stage=3;
		}
    break;
    case 3: // moving towards ball
		private_bb->SetDouble("faceTowardsPosx", passTo.x);
	    private_bb->SetDouble("faceTowardsPosy", passTo.y);
	    private_bb->SetDouble("radius", 0.1);
	    result = rotateAroundPoint.Update();
	    if (result == Status::Success) {
			return Status::Success;
		}
    break;
	}

    if (result == Status::Failure) {
		// ROS_INFO_STREAM("AimAtSafe failed :(");
		return Status::Failure;
	} else {
		return Status::Running;
	}
};

} // rtt
