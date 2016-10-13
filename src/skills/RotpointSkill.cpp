#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/RotpointSkill.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

	
namespace rtt {


	
	RotpointSkill::RotpointSkill(bt::Blackboard::Ptr blackboard):Skill(aggregator,blackboard){
	}
	
	double RotpointSkill::cleanAngle(double angle){
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
	
	
	roboteam_utils::Vector2 RotpointSkill::worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
        roboteam_utils::Vector2 robotRequiredv;
        
        robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
        robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
		
		return robotRequiredv;
    }
    
    void RotpointSkill::updateArgs(ros::Publisher pub, int robotID, double targetAngle, double w){
    	this->pub=pub;
    	this->robotID=robotID;
    	this->targetAngle=targetAngle;
    	this->rotw=w;
    }
    
	bt::Node::Status RotpointSkill::Update (){
		roboteam_msgs::World world = LastWorld::get();		
		roboteam_msgs::WorldBall ball = world.ball;

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
		
		roboteam_msgs::WorldRobot robot = world.robots_yellow[0];
	
		roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
		roboteam_utils::Vector2 posDiff = ballPos-robotPos;
		
		
		if (posDiff.length() < 0.105) { // got ball
			//if (posDiff.angle() < 0.05 and posDiff.angle() > -0.05){ // correct orientation
			
			
				// initially assume facing towards center and rotate around ball
				roboteam_utils::Vector2 center;
				center.x=ballPos.x;
				center.y=ballPos.y;
				// TODO: this will break in real world because exact distance to ball is not known
				double radius=0.102; 
		
				// velocity in towards and away from ball
				double Pconstant=3;
				double maxv=0.4;
				double scalefactor=(1 - radius/posDiff.length());
				roboteam_utils::Vector2 requiredv=posDiff.scale(scalefactor)*Pconstant;
				
				roboteam_utils::Vector2 robotrequiredv=worldToRobotFrame(requiredv, robot.w);
								
				if(robotrequiredv.x > maxv){
					robotrequiredv.x=robotrequiredv.x/requiredv.length()*maxv;
				}
				
				// velocity around ball
				Pconstant=2;
				
				double rotDiff=-targetAngle+robot.w;
				rotDiff=cleanAngle(rotDiff);
				double requiredrotv=rotDiff*Pconstant;
				if(requiredrotv > rotw){
					requiredrotv=rotw;
				}
				
				robotrequiredv.y=requiredrotv*radius;
				if(robotrequiredv.y > maxv){
					robotrequiredv.y=robotrequiredv.y/requiredv.length()*maxv;
				}
				
				
				// rotation controller
				double rotPconstant=6;
				double maxrotv=3.0;
				double estimateddelay=1;
				
				
				double requiredrot=posDiff.angle()-estimateddelay*robotrequiredv.y;
				double currentrot=robot.w;
				
				double rotdiff=cleanAngle(requiredrot-currentrot);
			   	        
				if (rotdiff > M_PI){rotdiff=M_PI-rotdiff;}
				
				requiredrotv=rotdiff*rotPconstant;
				
				if(requiredrotv>maxrotv){
					requiredrotv=maxrotv;
				}
				else if(requiredrotv<-maxrotv){
					requiredrotv=-maxrotv;
				}
				
						
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
			
				pub.publish(cmd);
				

				return Status::Running;
			/*}
			else {
				ROS_INFO("not faceing ball");
				return Status::Failure;
			}*/
		}
		else {
			//ROS_INFO("not close to ball");
			
			return Status::Failure;
		}
		
		
	}

} // rtt


