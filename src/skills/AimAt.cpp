#include <string>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/skills/AimAt.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

AimAt::AimAt(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , rotateAroundPoint(n, "", private_bb) {
	pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
}

bt::Node::Status AimAt::Update (){
	roboteam_msgs::World world = LastWorld::get();

	// Check is world contains a sensible message. Otherwise wait, it might the case that AimAt::Update 
	// is called before the first world state update
	int robotID = blackboard->GetInt("ROBOT_ID");
	std::string destination = GetString("At");
	// std::string destination=private_bb->GetString("At");
	// ROS_INFO_STREAM("destination1: " << destination);
	// printf("%s\n", destination1.c_str());

	if (world.us.size() == 0) {
		//ROS_INFO("No information about the world state :(");
		return Status::Running;
	}

	// roboteam_msgs::WorldBall ball = world.ball;
	// roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	//roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	
	roboteam_utils::Vector2 passTo;
	
	if (destination=="robot"){

		int AtRobotID = GetInt("AtRobot");
		roboteam_msgs::WorldRobot passTorobot=world.us.at(AtRobotID);
		passTo=roboteam_utils::Vector2(passTorobot.pos.x, passTorobot.pos.y);
	

	} else if(destination=="theirgoal"){
        std::string our_field_side = "left";
        n.getParam("our_field_side", our_field_side);

        auto field = LastWorld::get_field();

		if(our_field_side == "left"){
			passTo=roboteam_utils::Vector2(field.field_length/2.0, 0);

		} else {
			passTo=roboteam_utils::Vector2(field.field_length/-2.0, 0);
		}
	} else if(destination=="ourgoal"){
        std::string our_field_side = "left";
        n.getParam("our_field_side", our_field_side);

        auto field = LastWorld::get_field();

		if(our_field_side == "right"){
			passTo=roboteam_utils::Vector2(field.field_length/2.0, 0);
		} else {
			passTo=roboteam_utils::Vector2(field.field_length/-2.0, 0);
		}
	}
	
	// ROS_INFO("passto: x:%f, y:%f",passTo.x, passTo.y);
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetString("center", "ball");
    private_bb->SetDouble("faceTowardsPosx", passTo.x);
    private_bb->SetDouble("faceTowardsPosy", passTo.y);
    private_bb->SetDouble("w",3.0);
    private_bb->SetDouble("radius", 0.1);

	
	return rotateAroundPoint.Update();
};

} // rtt
