#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/skills/DribbleCurve.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"


namespace rtt {
	DribbleCurve::DribbleCurve(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(n, name, blackboard)
            , rotateAroundPoint(n, "", private_bb) { 
        pubDribbleCurve = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
        ROS_INFO("Dribbling");
	}

void DribbleCurve::stoprobot(int robotID) {

	roboteam_msgs::RobotCommand cmd;
	cmd.id = robotID;
	cmd.active = true;
	cmd.x_vel = 0.0;
	cmd.y_vel = 0.0;
	cmd.w = 0.0;

	cmd.dribbler=true;
	cmd.kicker=false;
	cmd.kicker_vel=0.0;
	cmd.kicker_forced=false;
	cmd.chipper=false;
	cmd.chipper_vel=0.0;
	cmd.chipper_forced=false;

	pubDribbleCurve.publish(cmd);
}
roboteam_utils::Vector2 DribbleCurve::worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double DribbleCurve::computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos; 
	return differenceVector.angle();
}

double DribbleCurve::cleanAngle(double angle){
	if (angle < M_PI){
		return fmod(angle-M_PI, (2*M_PI))+M_PI;
	}
	else if(angle > M_PI){
		return fmod(angle+M_PI, (2*M_PI))-M_PI;
	}
	else {
		return angle;
	}
}


bt::Node::Status DribbleCurve::Update() {
	// development
	int robotID=0;
	
	roboteam_msgs::World world = LastWorld::get();
    //int robotID = blackboard->GetInt("ROBOT_ID");

	roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update 
	// is called before the first world state update
	if (world.us.size() == 0) {
		ROS_INFO("No information about the world state :(");
		return Status::Running;
	}

    // TODO: Even though you checked the size here, you should still use std::vector::at()!
	roboteam_msgs::WorldRobot robot = world.us[robotID];
	
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	
	/*
	private_bb->SetInt("ROBOT_ID", robotID);
	private_bb->SetString("center", "point");
	private_bb->SetDouble("centerx", goalPos.x);
	private_bb->SetDouble("centery", goalPos.y);
	private_bb->SetDouble("radius", goalposDiff.length()+0.1);
	private_bb->SetDouble("faceTowardsPosx", facetowardsPos.x);
	private_bb->SetDouble("faceTowardsPosy", facetowardsPos.y);
	private_bb->SetDouble("w", 3.0);
		
	rotateAroundPoint.Update();
	*/
	
	
	return Status::Running;
}

} // rtt




