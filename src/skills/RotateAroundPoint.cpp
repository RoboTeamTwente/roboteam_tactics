#include <cstring>
#include <iostream>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
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
	cmd.w = 0.0;

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
	// int:ROBOT_ID=
	// string:center="ball"					//
	// double:faceTowardsPosx=				//
	// double:faceTowardsPosy=				//
	// double:w= (rotation speed (pick 3)	//
	
	// -- or --
	
	// int:ROBOT_ID=
	// string:center="point"				//
	// double:centerx=
	// double:centery=
	// double:radius=
	// double:faceTowardsPosx=				//
	// double:faceTowardsPosy=				//
	// double:w= (rotation speed (pick 2))	//
	//**************************************//
	
	
	
	// int robotID2 = GetInt("ROBOT_ID");

	// check and set ROBOT_ID
	if(blackboard->HasInt("ROBOT_ID")){ 
		robotID = blackboard->GetInt("ROBOT_ID");
	}
	else {
		ROS_INFO("No int:ROBOT_ID specified"); 
		return Status::Failure;
	}

	// get world, robot and ball
	roboteam_msgs::World world = LastWorld::get();
	
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
	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_msgs::WorldBall ball = world.ball;
	
	
	// check and set other settings
	
	if(HasDouble("faceTowardsPosx") and HasDouble("faceTowardsPosy")){
		faceTowardsPos=roboteam_utils::Vector2(
			GetDouble("faceTowardsPosx"),
			GetDouble("faceTowardsPosy")
		);
	}
	else {
		ROS_INFO("No double:facetowardsPos x and y specified"); 
		return Status::Failure;
	}
	
	if(HasDouble("w")){
		rotw = GetDouble("w");
	}
	else {
		ROS_INFO("No rot velocity double:w specified"); 
		return Status::Failure;
	}
	
	if(HasString("center")){
		if(GetString("center")=="ball"){
			center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
			radius = 0.1;
		} 
		else if(GetString("center")=="point"){
			if(HasDouble("centerx") and HasDouble("centery")){
				center = roboteam_utils::Vector2(
					GetDouble("centerx"),
					GetDouble("centery")
				);
			}
			else {
				ROS_INFO("no double:center x and y specified");
				return Status::Failure;
			}
			
			if(HasDouble("radius")){
				radius = GetDouble("radius");
			}
			else {
				ROS_INFO("no double:radius specified");
				return Status::Failure;
			}
			
		}
		else {
			ROS_INFO("unknown string:center choice specified, use ball or point");
			return Status::Failure;
		}
	}
	else {
		ROS_INFO("no string:center choice specified ball|point"); 
		return Status::Failure;
	}
	    
	
	// vector calculations
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	double targetAngle=computeAngle(robotPos, faceTowardsPos);

	roboteam_utils::Vector2 worldposDiff = center-robotPos;
	roboteam_utils::Vector2 targetVector = roboteam_utils::Vector2(radius*cos(targetAngle),radius*sin(targetAngle));
	roboteam_utils::Vector2 targetPos=targetVector+center;

	// ------------------------------ //
	//
	// OH NO, unused variable!!!! 
	double worldrottoballdiff=cleanAngle(worldposDiff.angle()-robot.angle);
	// aaaaaaaaah, sooo many warnings
	//
	// ------------------------------ //

	double worldrotDiff=(robotPos-center).angle()-(targetAngle+M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	if (worldposDiff.length() < 1.5*radius) { // close enough
		//if (worldrottoballdiff < 1 and worldrottoballdiff > -1){ // oriented towards center		
		if(true){
			roboteam_utils::Vector2 radiusdirection=center-robotPos;
			radiusdirection=radiusdirection.normalize();
			roboteam_utils::Vector2 turndirection(radiusdirection.y, -radiusdirection.x); // perpendicular;
			turndirection=turndirection.normalize();
			
			
			roboteam_utils::Vector2 robotrequiredv;
			roboteam_utils::Vector2 worldrequiredv;
			
			// velocity in towards and away from ball (x) and around ball (y)
			double radiusPconstant=10;
			double turnPconstant=5;
			double maxv=5.0;
			double maxrot=4.0;

			
			double radiusDiff=(worldposDiff.length()-radius);
			double radiusReq=radiusDiff*radiusPconstant;
			
			radiusdirection=radiusdirection.scale(radiusReq);
			
			
			double turnDiff=worldrotDiff;
			double turnReq=turnDiff*turnPconstant;
								
			if(turnReq > rotw){
				turnReq=rotw;
			}
			if(turnReq < -rotw){
				turnReq=-rotw;
			}
			
			turnReq*=radius;

			turndirection=turndirection.scale(-turnReq);
			
			robotrequiredv=worldToRobotFrame(radiusdirection+turndirection, robot.angle);
					
			roboteam_utils::Vector2 extrav(GetDouble("extravx"),GetDouble("extravy"));
			robotrequiredv=robotrequiredv+extrav;
				
			if(robotrequiredv.length() > maxv){
				robotrequiredv=robotrequiredv/robotrequiredv.length()*maxv;
			}
			
			// rotation controller (w)
			double rotPconstant=20;
			
			double reqRobotrot=worldposDiff.angle();
			double robotrot=robot.angle;
			double reqRobotrotDiff=cleanAngle(reqRobotrot-robotrot);
		   	        
			if (reqRobotrotDiff > M_PI){reqRobotrotDiff=M_PI-reqRobotrotDiff;}
	
			double requiredrotv=reqRobotrotDiff*rotPconstant;
			if(requiredrotv > maxrot){requiredrotv=maxrot;}
			if(requiredrotv < -maxrot){requiredrotv=-maxrot;}

			
			if (extrav.x > 0.01 or extrav.y>0.01 or fabs(worldrotDiff) > 0.005 or fabs(radiusReq) > 0.1 or fabs(turnReq) > 0.1) { // robot not finished yet
			
				// send command
				roboteam_msgs::RobotCommand cmd;
				cmd.id = robotID;
				cmd.active = true;
				cmd.x_vel = robotrequiredv.x;
				cmd.y_vel = robotrequiredv.y;
				cmd.w = requiredrotv;
				//cmd.w=0.0;
				
				cmd.dribbler=true;
				cmd.kicker=false;
				cmd.kicker_vel=0.0;
				cmd.kicker_forced=false;
				cmd.chipper=false;
				if(extrav.x>0.01 or extrav.y>0.01){
					cmd.kicker_vel=0.01;
				}
				cmd.chipper_vel=0.0;
				cmd.chipper_forced=false;
				//ROS_INFO("rotDiff:%f cmd vel x:%f, y:%f, w:%f", worldrotDiff ,cmd.x_vel,cmd.y_vel,cmd.w);
				pub.publish(cmd);
	
				return Status::Running;
				
			} else { // final position reached
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
