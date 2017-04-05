#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include <stdlib.h>
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/skills/Dribble.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"


namespace rtt {

RTT_REGISTER_SKILL(Dribble);

Dribble::Dribble(std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(name, blackboard)
            , rotateAroundPoint("", private_bb) { 
        pubDebugpoints = n.advertise<roboteam_msgs::DebugPoint>(TOPIC_DEBUG_POINTS, 1000);
        ROS_INFO("Dribbling");
	}

void Dribble::stoprobot(int robotID) {
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(stop_command(robotID));
}
Vector2 Dribble::worldToRobotFrame(Vector2 requiredv, double rotation){
    Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double Dribble::computeAngle(Vector2 robotPos, Vector2 faceTowardsPos) {
	Vector2 differenceVector = faceTowardsPos - robotPos; 
	return differenceVector.angle();
}

double Dribble::cleanAngle(double angle){
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

Vector2 Dribble::saveDribbleDeceleration(Vector2 reqspeed){
	// TODO: should improve timing accuracy, because rate is not really known.
	Vector2 deceleration=prevspeed-reqspeed;
	double timestep=1.0/60;
	double maxdeceleration=1.0;
	Vector2 speed;
	if(deceleration.length() > timestep*maxdeceleration){
		speed=prevspeed-deceleration.normalize()*maxdeceleration*timestep;
	}
	else {
		speed=reqspeed;
	}
	
	prevspeed=speed;
	
	return speed;
}


bt::Node::Status Dribble::Update() {
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
	
	Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
	Vector2 goalPos = Vector2(
					GetDouble("goalx"),
					GetDouble("goaly")
				);
	
	Vector2 goalposDiff = ballPos-goalPos;
	
	double targetAngle=computeAngle(robotPos, goalPos);
	double worldrotDiff=(robotPos-ballPos).angle()-(targetAngle+M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	double worldrottoballdiff=cleanAngle(goalposDiff.angle()-robot.angle+M_PI);
	
	if(goalposDiff.length() > 0.02){
		if((robotPos-ballPos).length() > 0.2){
			ROS_INFO("lost ball");
			stoprobot(robotID);
			return Status::Failure;
		}
		
		Vector2 rotatearoundPos;
		
		if (worldrotDiff > 20.0/180.0*M_PI or worldrotDiff < -20.0/180.0*M_PI){ // oriented towards goal behind ball	
			ROS_INFO("rotate around ball");
			rotatearoundPos=ballPos;
		}
		else {
			rotatearoundPos=goalPos-goalposDiff*(fabs(worldrottoballdiff)/45.0);
		}
	
		Vector2 facetowardsPos=goalPos*2-ballPos;
		
		double radius=(rotatearoundPos-robotPos).length();
		// debug points;
		roboteam_msgs::DebugPoint debugpoint;
		debugpoint.name="rotatearound";
		debugpoint.remove=false;
		debugpoint.pos.x=rotatearoundPos.x;
		debugpoint.pos.y=rotatearoundPos.y;
		debugpoint.color.r=255;
		debugpoint.color.g=0;
		debugpoint.color.b=0;
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
				
		if(fabs(worldrotDiff) < 45.0/180.0*M_PI){
			maxv=maxv*(45.0/180.0*M_PI-worldrotDiff)/(45.0/180.0*M_PI);
		}
		else {
			maxv=0.0;
		}

		double vPconstant=1.5;
	
		Vector2 vtogoal=goalposDiff*-vPconstant;
		
	
	
		if(vtogoal.length() > maxv){
			vtogoal=vtogoal/vtogoal.length()*maxv;
		}
		
		
		robotvtogoal=worldToRobotFrame(vtogoal, robot.angle);
			
		debugpoint.name="oldrobotvtogoal";
		debugpoint.remove=false;
		debugpoint.pos.x=robotvtogoal.x;
		debugpoint.pos.y=robotvtogoal.y;
		debugpoint.color.r=0;
		debugpoint.color.g=0;
		debugpoint.color.b=255;
		pubDebugpoints.publish(debugpoint);
		
		

		robotvtogoal=saveDribbleDeceleration(robotvtogoal);
		
		debugpoint.name="robotvtogoal";
		debugpoint.remove=false;
		debugpoint.color.r=0;
		debugpoint.color.g=0;
		debugpoint.color.b=255;
		debugpoint.pos.x=robotvtogoal.x;
		debugpoint.pos.y=robotvtogoal.y;
		pubDebugpoints.publish(debugpoint);
		
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




