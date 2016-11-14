#include "ros/ros.h"
#include <stdlib.h>
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
        pubDebugpoints = n.advertise<roboteam_msgs::Vector2f>("view_debug_points", 1000);
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
	roboteam_msgs::World world = LastWorld::get();
    int robotID = blackboard->GetInt("ROBOT_ID");

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
	roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(
					GetDouble("goalx"),
					GetDouble("goaly")
				);
	roboteam_utils::Vector2 goal2Pos = roboteam_utils::Vector2(
					GetDouble("goal2x"),
					GetDouble("goal2y")
				);
				
	roboteam_utils::Vector2 goalposDiff = ballPos-goalPos;
	
	if (goal1reached==false and goalposDiff.length() < 0.5){ // go to second goal	
		goal1reached=true;
		ROS_INFO("goal1reached");
	}
	if(goal1reached){
		goalPos=goal2Pos;
		goalposDiff=ballPos-goalPos;
	}
	
	
	//roboteam_utils::Vector2 ballposDiff = robotPos-ballPos;	
	double targetAngle=computeAngle(robotPos, goalPos);
	double worldrotDiff=(robotPos-ballPos).angle()-(targetAngle+M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	double worldrottoballdiff=cleanAngle(goalposDiff.angle()-robot.angle+M_PI);
	
	ROS_INFO("worldrotDiff: %f",worldrotDiff);
	if(goalposDiff.length() > 0.02){
		if((robotPos-ballPos).length() > 0.5){
			ROS_INFO("lost ball");
			return Status::Failure;
		}
		bool startdriving=false;
		
		roboteam_utils::Vector2 rotatearoundPos;
		
		if (worldrotDiff > 45.0/180.0*M_PI or worldrotDiff < -45.0/180.0*M_PI){ // oriented towards goal behind ball	
			rotatearoundPos=ballPos;
		}
		else {
			rotatearoundPos=goalPos-goalposDiff*(fabs(worldrottoballdiff)/45.0);
		}

		roboteam_utils::Vector2 facetowardsPos=goalPos*2-ballPos;
		//ROS_INFO("behind ball");
		
		double radius=(rotatearoundPos-ballPos).length()+0.1;
		
		// debug points;
		roboteam_msgs::Vector2f debugpoint;
		debugpoint.x=rotatearoundPos.x;
		debugpoint.y=rotatearoundPos.y;
		pubDebugpoints.publish(debugpoint);
		
		debugpoint.x=facetowardsPos.x;
		debugpoint.y=facetowardsPos.y;
		pubDebugpoints.publish(debugpoint);
		
		private_bb->SetInt("ROBOT_ID", robotID);
		private_bb->SetString("center", "point");
		private_bb->SetDouble("centerx", rotatearoundPos.x);
		private_bb->SetDouble("centery", rotatearoundPos.y);
		private_bb->SetDouble("radius", radius);
		private_bb->SetDouble("faceTowardsPosx", facetowardsPos.x);
		private_bb->SetDouble("faceTowardsPosy", facetowardsPos.y);
		private_bb->SetDouble("w", 3.0);
	
		double maxv=1.5;
		// the higher worldrotDiff, the lower maxv
	
		if(fabs(worldrotDiff) > 0.1){
			maxv=maxv*0.5/(fabs(worldrotDiff));
		}
	
		double vPconstant=1.5;
	
		roboteam_utils::Vector2 vtogoal=goalposDiff*-vPconstant;
		
	
	
		if(vtogoal.length() > maxv){
			vtogoal=vtogoal/vtogoal.length()*maxv;
		}
		robotvtogoal=worldToRobotFrame(vtogoal, robot.angle);
		/*
		ROS_INFO("robotvtogoal x:%f, y:%f",robotvtogoal.x, robotvtogoal.y);
		
		roboteam_utils::Vector2 diffPreviousRobotvtogoal=robotvtogoal-prevRobotvtogoal;
		if(diffPreviousRobotvtogoal.length() > 0.1){
			double timestep=1.0/60;
			robotvtogoal=robotvtogoal+diffPreviousRobotvtogoal.normalize()/5*timestep;
		
		}
		prevRobotvtogoal=robotvtogoal;
		*/
		private_bb->SetDouble("extravx",robotvtogoal.x);
		private_bb->SetDouble("extravy",robotvtogoal.y);
		
	
		
	}
	else {
		stoprobot(robotID);
		return Status::Success;
		
	}
	rotateAroundPoint.Update();
	
	return Status::Running;
}

} // rtt




