#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
	
RotateAroundPoint::RotateAroundPoint(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {}

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

void RotateAroundPoint::updateArgs(ros::Publisher pub, int robotID, roboteam_utils::Vector2 faceTowardsPos, double w, roboteam_utils::Vector2 center, double radius){
	this->pub=pub;
	this->robotID=robotID;
	// this->targetAngle=targetAngle;
	this->faceTowardsPos=faceTowardsPos;
	this->rotw=w;
	this->center=center;
	this->radius=radius;
}

void RotateAroundPoint::computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos; 
	targetAngle = differenceVector.angle();
}

void RotateAroundPoint::stoprobot(ros::Publisher pub, int robotID) {

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
	roboteam_msgs::World world = LastWorld::get();
	if (world.robots_yellow.size() == 0){
		return Status::Running;
	}
	if (world.header.seq==prevworldseq and !firstworld){
		return Status::Running;
	}
	else {
		firstworld=false;
		prevworldseq=world.header.seq;
	}
	
	roboteam_msgs::WorldBall ball = world.ball;
	roboteam_msgs::WorldRobot robot = world.robots_yellow[0];
	center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	radius = 0.09;

	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 worldposDiff = center-robotPos;
	roboteam_utils::Vector2 targetVector = roboteam_utils::Vector2(radius*cos(targetAngle),radius*sin(targetAngle));
	roboteam_utils::Vector2 targetPos=targetVector+center;
	
	double worldrottoballdiff=cleanAngle(worldposDiff.angle()-robot.w);
	
	double worldrotDiff=(robotPos-center).angle()-(targetAngle-M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	
	if (worldposDiff.length() < 1.5*radius) { // close enough
		if (worldrottoballdiff < 1 and worldrottoballdiff > -1){ // oriented towards center		
			// velocity in towards and away from ball (x)
			double Pconstant=3.0;
			double maxv=5.0;
			double scalefactor=(1 - radius/worldposDiff.length());
			roboteam_utils::Vector2 requiredv=worldposDiff.scale(scalefactor)*Pconstant;
	
			roboteam_utils::Vector2 robotrequiredv=worldToRobotFrame(requiredv, robot.w);
					
			if(robotrequiredv.x > maxv){
				robotrequiredv.x=robotrequiredv.x/requiredv.length()*maxv;
			}
	
			// velocity around ball (y)
			Pconstant=20;
	
			double reqWorldrotV=worldrotDiff*Pconstant;
			if(reqWorldrotV > rotw){
				reqWorldrotV=rotw;
			}
			if(reqWorldrotV < -rotw){
				reqWorldrotV=-rotw;
			}

			robotrequiredv.y=reqWorldrotV*radius;
			if(robotrequiredv.y > maxv){
				robotrequiredv.y=robotrequiredv.y/robotrequiredv.length()*maxv;
			}
	
			// rotation controller (w)
			double rotPconstant=10;
			double estimateddelay=10;

			double reqRobotrot=worldposDiff.angle();
			double robotrot=robot.w;
	
			double reqRobotrotDiff=cleanAngle(reqRobotrot-robotrot);
		   	        
			if (reqRobotrotDiff > M_PI){reqRobotrotDiff=M_PI-reqRobotrotDiff;}
	
			double requiredrotv=reqRobotrotDiff*rotPconstant+estimateddelay*-robotrequiredv.y;
			
			ROS_INFO_STREAM(reqRobotrotDiff);
			if (requiredv.x > 0.01 or requiredv.y > 0.01 or requiredrotv > 0.1){ // robot not finished yet
			
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
				
				stoprobot(pub,robotID);
				ROS_INFO("finished");
				return Status::Success;
			}
		}
		else { // wrong orientation 
			ROS_INFO("not oriented enough toward center");
			stoprobot(pub,robotID);
			return Status::Failure;
		}
	}
	else {
		ROS_INFO("not close enough to turn circle (center+radius)");
		stoprobot(pub,robotID);
		return Status::Failure;
	}
}

} // rtt