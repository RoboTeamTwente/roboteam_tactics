#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include <cstring>
#include <iostream>
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
	
RotateAroundPoint::RotateAroundPoint(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {

	pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
        
}

double RotateAroundPoint::cleanAngle(double angle){
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

roboteam_utils::Vector2 RotateAroundPoint::worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double RotateAroundPoint::computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos; 
	return differenceVector.angle();
}

void RotateAroundPoint::stoprobot(int robotID) {

	roboteam_msgs::RobotCommand cmd;
	cmd.id = robotID;
	cmd.active = true;
	cmd.x_vel = 0.0;
	cmd.y_vel = 0.0;
	cmd.w_vel = 0.0;

	cmd.dribbler=true;
	cmd.kicker=false;
	cmd.kicker_vel=0.0;
	cmd.kicker_forced=false;
	cmd.chipper=false;
	cmd.chipper_vel=0.0;
	cmd.chipper_forced=false;

	pub.publish(cmd);
}

bt::Node::Status RotateAroundPoint::Update (){
	//**************************************//
	// NEEDS BLACKBOARD ARGUMENTS: 			//
	
	// string:center="ball"					//
	// double:faceTowardsPosx=				//
	// double:faceTowardsPosy=				//
	// double:w= (rotation speed (pick 3)	//
	
	// -- or --
	
	// string:center="point"				//
	// double:centerx=
	// double:centery=
	// double:radius=
	// double:faceTowardsPosx=				//
	// double:faceTowardsPosy=				//
	// double:w= (rotation speed (pick 2))	//
	
	
	//**************************************//
	
	
	
	
	// get world
	roboteam_msgs::World world = LastWorld::get();


    // TODO: You should use at() here instead of []! And check the bounds as well!
    // What if us contains no robots?
	
	if (world.us.size() == 0){

		return Status::Running;
	}
	if (world.header.seq==prevworldseq and !firstworld){
		return Status::Running;
	}
	else {
		firstworld=false;
		prevworldseq=world.header.seq;
	}
	roboteam_msgs::WorldRobot robot = world.us.at(0);
	
	roboteam_msgs::WorldBall ball = world.ball;
	
	// settings
	
	
	
	roboteam_utils::Vector2 faceTowardsPos(GetDouble("faceTowardsPosx"),GetDouble("faceTowardsPosy"));
	
    double rotw = GetDouble("w");
    int robotID = blackboard->GetInt("ROBOT_ID");
    //double radius = GetDouble("radius");
		
   	if(GetString("center")=="ball"){
		center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		radius = 0.1;
   			
   	} else if(GetString("center")=="point"){ROS_INFO("yo");
		std::cout << private_bb->GetString("centerx");
		center = roboteam_utils::Vector2(GetDouble("centerx"),GetDouble("centery"));
		radius = GetDouble("radius");
   	
   	}
   	else {
   		ROS_INFO("ERR no center specified in blackboard");
   	}
    
    
	
	// calculations

	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 worldposDiff = center-robotPos;
	roboteam_utils::Vector2 targetVector = roboteam_utils::Vector2(radius*cos(targetAngle),radius*sin(targetAngle));
	roboteam_utils::Vector2 targetPos=targetVector+center;

	targetAngle=computeAngle(robotPos, faceTowardsPos);

	double worldrottoballdiff=cleanAngle(worldposDiff.angle()-robot.angle);
	double worldrotDiff=(robotPos-center).angle()-(targetAngle);
	worldrotDiff=cleanAngle(worldrotDiff);
	
	if (worldposDiff.length() < 1.5*radius) { // close enough
		if (worldrottoballdiff < 1 and worldrottoballdiff > -1){ // oriented towards center		
			roboteam_utils::Vector2 robotrequiredv;
			
			// velocity in towards and away from ball (x)
			double Pconstant=3.0;
			double maxv=5.0;
			double scalefactor=(1 - radius/worldposDiff.length());
			double radiusDiff=worldToRobotFrame(worldposDiff.scale(scalefactor), robot.angle).x;
			robotrequiredv.x=radiusDiff*Pconstant;
					
			// velocity around ball (y)
			Pconstant=10;
	
			double reqWorldrotV=worldrotDiff*Pconstant;
			if(reqWorldrotV > rotw){
				reqWorldrotV=rotw;
			}
			if(reqWorldrotV < -rotw){
				reqWorldrotV=-rotw;
			}

			robotrequiredv.y=reqWorldrotV*radius;
			
			
			
			
			// rotation controller (w)
			double rotPconstant=10;
			double estimateddelay=10;

			double reqRobotrot=worldposDiff.angle();
			double robotrot=robot.angle;
	
			double reqRobotrotDiff=cleanAngle(reqRobotrot-robotrot);
		   	        
			if (reqRobotrotDiff > M_PI){reqRobotrotDiff=M_PI-reqRobotrotDiff;}
	
			double requiredrotv=reqRobotrotDiff*rotPconstant+estimateddelay*-robotrequiredv.y;
			
			// check for max
			if(robotrequiredv.length() > maxv){
				robotrequiredv=robotrequiredv/robotrequiredv.length()*maxv;
			}
			
			
			
			if (fabs(worldrotDiff) > 0.01 or fabs(radiusDiff) > 0.1 or fabs(reqRobotrotDiff) > 0.1) { // robot not finished yet
			
				// send command
				roboteam_msgs::RobotCommand cmd;
				cmd.id = robotID;
				cmd.active = true;
				cmd.x_vel = robotrequiredv.x;
				cmd.y_vel = robotrequiredv.y;
				cmd.w_vel = requiredrotv;

				cmd.dribbler=true;
				cmd.kicker=false;
				cmd.kicker_vel=0.0;
				cmd.kicker_forced=false;
				cmd.chipper=false;
				cmd.chipper_vel=0.0;
				cmd.chipper_forced=false;
				ROS_INFO("rotDiff:%f cmd vel x:%f, y:%f, w:%f", worldrotDiff ,cmd.x_vel,cmd.y_vel,cmd.w_vel);
				pub.publish(cmd);
	
				return Status::Running;
				
			} 
			else { // final position reached
				// TODO: maybe instead call position controller
				
				stoprobot(robotID);
				ROS_INFO("finished");
				return Status::Success;
			}
		}
		else { // wrong orientation 
			ROS_INFO("not oriented enough toward center");
			stoprobot(robotID);
			return Status::Failure;
		}
	}
	else {
		ROS_INFO("not close enough to turn circle (center+radius)");
		stoprobot(robotID);
		return Status::Failure;
	}
}

} // rtt
