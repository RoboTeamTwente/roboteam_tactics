#include "roboteam_tactics/treegen/LeafRegister.h"
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
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

namespace rtt {
	
RTT_REGISTER_SKILL(RotateAroundPoint);

RotateAroundPoint::RotateAroundPoint(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard),
        goto_bb(std::make_shared<bt::Blackboard>()), 
        goToPos("", goto_bb) {
	}

void RotateAroundPoint::stoprobot(int robotID) {

    if (HasBool("quiet") && GetBool("quiet")) {
        return;
    }

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

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(cmd);
}

bt::Node::Status RotateAroundPoint::checkAndSetArguments(){
	
	
	// check and set other settings
	
	if(HasDouble("faceTowardsPosx") and HasDouble("faceTowardsPosy")){
		faceTowardsPos=Vector2(
			GetDouble("faceTowardsPosx"),
			GetDouble("faceTowardsPosy")
		);
	} else {
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
			center = Vector2(ball.pos.x, ball.pos.y);
			if(HasDouble("radius")){
				radius = GetDouble("radius");
			}
			else {
				radius=0.1;
			}
		} 
		else if(GetString("center")=="point"){
			if(HasDouble("centerx") and HasDouble("centery")){
				center = Vector2(
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

	if(HasDouble("rotPconstant")){
		rotPconstant = GetDouble("rotPconstant");
	}
	else {
		rotPconstant=5.0;
	}

	if(HasDouble("radiusPconstant")){
		radiusPconstant = GetDouble("radiusPconstant");
	}
	else {
		radiusPconstant=4.0;
	}

	if(HasDouble("turnPconstant")){
		turnPconstant = GetDouble("turnPconstant");
	}
	else {
		turnPconstant=4.0;
	}
	return Status::Running;

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


	// -- extra ---

	// bool:faceoutward=true
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

    if (!LastWorld::have_received_first_world()) {
        return Status::Running;
    }

	// if (world.header.seq==prevworldseq and !firstworld){
        // std::cout << "Same world seq!\n";
		// return Status::Running;
	// }
	// else {
		// firstworld=false;
		// prevworldseq=world.header.seq;
	// }
    
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("DefendGoalarea could not find robot");
        return Status::Failure;
    }

	ball = world.ball;

	Status status = checkAndSetArguments();
	if(status != Status::Running){
		return status;
	}

	// vector calculations
	Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	double targetAngle=computeAngle(robotPos, faceTowardsPos);

	bool faceoutward=false;
	if(HasBool("faceoutward") && GetBool("faceoutward")){
		faceoutward=true;
		targetAngle=cleanAngle(targetAngle+M_PI);
	}

	Vector2 worldposDiff = center-robotPos;

	double worldrotDiff=(robotPos-center).angle()-(targetAngle+M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	//if (worldposDiff.length() < 1.5*radius) { // close enough
	if (true){
		//if (worldrottoballdiff < 1 and worldrottoballdiff > -1) // oriented towards center		
		if(true){
		
			Vector2 radiusdirection=center-robotPos;
			radiusdirection=radiusdirection.normalize();
			Vector2 turndirection(radiusdirection.y, -radiusdirection.x); // perpendicular;
			turndirection=turndirection.normalize();
			
			
			Vector2 robotrequiredv;
			Vector2 worldrequiredv;
			
			// velocity in towards and away from ball (x) and around ball (y)
			
			double maxv= HasDouble("maxv") ? GetDouble("maxv") : 1.0;
			double maxrot=4.0;

			
			double radiusDiff=(worldposDiff.length() - radius);
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
			
            // ROS_INFO("----DEBUGGING-----");
            // ROS_INFO("robotrequiredv x:%f, y:%f", robotrequiredv.x,robotrequiredv.y);
			
			Vector2 extrav(GetDouble("extravx"),GetDouble("extravy"));
			robotrequiredv=robotrequiredv+extrav;

            // ROS_INFO("robotrequiredv after: x:%f, y:%f", robotrequiredv.x,robotrequiredv.y);
			
			bool forceExtrav=false;
			if(HasBool("forceExtrav") && GetBool("forceExtrav")){ // for demonstration purposes
				forceExtrav=true;

			}

			if(robotrequiredv.length() > maxv && !forceExtrav){
				robotrequiredv=robotrequiredv/robotrequiredv.length()*maxv;
			}

            // ROS_INFO("robotrequiredv after max: x:%f, y:%f", robotrequiredv.x,robotrequiredv.y);
			
            /////////////////////////////
			// rotation controller (w) //
            /////////////////////////////
		
			double reqRobotrot=worldposDiff.angle();

			if(faceoutward){
				reqRobotrot=reqRobotrot+M_PI;
			}

			double robotrot=robot.angle;
			double reqRobotrotDiff=cleanAngle(reqRobotrot-robotrot);
		   	        
			if (reqRobotrotDiff > M_PI){reqRobotrotDiff=M_PI-reqRobotrotDiff;}
	
			double requiredrotv=reqRobotrotDiff*rotPconstant;
			if(requiredrotv > maxrot){requiredrotv=maxrot;}
			if(requiredrotv < -maxrot){requiredrotv=-maxrot;}

			double successDist=0.13;
			if(HasDouble("successDist")){
				successDist=GetDouble("successDist");
			}

			double successAngle=0.005;
			if(HasDouble("successAngle")){
				successDist=GetDouble("successAngle");
			}


			// ROS_INFO_STREAM("worldrotDiff: " << worldrotDiff);
			if (fabs(worldrotDiff) > successAngle or fabs(radiusReq) > successDist) { // robot not finished yet
				// send command
				roboteam_msgs::RobotCommand cmd;
				cmd.id = robotID;
				cmd.active = true;
				cmd.x_vel = robotrequiredv.x + 0.4;
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
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(cmd);
	
				return Status::Running;
				
			} else { // final position reached
				// TODO: maybe instead call position controller
				
				stoprobot(robotID);
				// ROS_INFO("RotateAroudPoint finished");
				return Status::Success;
			}
		}
		else { // wrong orientation 
			ROS_INFO("not oriented enough toward center");
			stoprobot(robotID);
			return Status::Failure;
		}
	} else {

		return Status::Failure;
        std::string our_color = "yellow";
        get_PARAM_OUR_COLOR(our_color);
		ROS_INFO_STREAM(
                "ID: "
                << std::to_string(robotID)
                << ", color: "
                << our_color
                << ": not close enough to turn circle (center+radius)");

		if(HasBool("drivetocircle") && GetBool("drivetocircle")){
			ROS_INFO("driving to circle");
			goto_bb->SetInt("ROBOT_ID", robotID);
			goto_bb->SetDouble("xGoal", center.x);
			goto_bb->SetDouble("angleGoal", targetAngle);
			goto_bb->SetDouble("yGoal", center.y);
			goto_bb->SetBool("avoidRobots", true);
			goto_bb->SetBool("dribbler", false);
			goToPos.Update();
			return Status::Running;
		}
		else {
			stoprobot(robotID);
			// ROS_INFO_STREAM("RotateAroudPoint failing...");
			return Status::Failure;
		}
	}
}

} // rtt
