#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/Dribble.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"


namespace rtt {
	Dribble::Dribble(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(n, name, blackboard)
            , rotateAroundPoint(n, "", private_bb) { 
        pubDribble = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
        ROS_INFO("Dribbling");
	}

void Dribble::stoprobot(int robotID) {

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

	pubDribble.publish(cmd);
}
roboteam_utils::Vector2 Dribble::worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double Dribble::computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos; 
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


bt::Node::Status Dribble::Update() {
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
	roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(
					GetDouble("goalx"),
					GetDouble("goaly")
				);
				
	roboteam_utils::Vector2 goalposDiff = ballPos-goalPos;
	//roboteam_utils::Vector2 ballposDiff = robotPos-ballPos;	
	double targetAngle=computeAngle(robotPos, goalPos);
	double worldrotDiff=(robotPos-ballPos).angle()-(targetAngle+M_PI);
	worldrotDiff=cleanAngle(worldrotDiff);
	double worldrottoballdiff=cleanAngle(goalposDiff.angle()-robot.angle+M_PI);
	
	if(goalposDiff.length() > 0.02){
	
		bool startdriving=false;
		if (worldrottoballdiff < 0.2 and worldrottoballdiff > -0.2){ // oriented towards goal behind ball	
			startdriving=true;
		}
		
		if(worldrottoballdiff > 0.5 or worldrottoballdiff < -0.5){
			startdriving=false;
		}
		
		
		
		if(startdriving){
			roboteam_utils::Vector2 facetowardsPos=goalPos*2-ballPos;
			ROS_INFO("behind ball");
			
			private_bb->SetInt("ROBOT_ID", robotID);
			private_bb->SetString("center", "point");
			private_bb->SetDouble("centerx", goalPos.x);
			private_bb->SetDouble("centery", goalPos.y);
			private_bb->SetDouble("radius", goalposDiff.length()+0.1);
			private_bb->SetDouble("faceTowardsPosx", facetowardsPos.x);
			private_bb->SetDouble("faceTowardsPosy", facetowardsPos.y);
			private_bb->SetDouble("w", 3.0);
		
			double maxv=1.5;
			// the higher worldrotDiff, the lower maxv
		
			if(fabs(worldrotDiff) > 0.1){
				maxv=maxv*0.1/(fabs(worldrotDiff));
			}
		
			double vPconstant=2;
		
			roboteam_utils::Vector2 vtogoal=goalposDiff*-vPconstant;
		
		
		
			if(vtogoal.length() > maxv){
				vtogoal=vtogoal/vtogoal.length()*maxv;
			}
		
			roboteam_utils::Vector2 robotvtogoal=worldToRobotFrame(vtogoal, robot.angle);
		
			private_bb->SetDouble("extravx",robotvtogoal.x);
			private_bb->SetDouble("extravy",robotvtogoal.y);
		
		}
		else { // first rotate around ball
			ROS_INFO("not behind ball");
			private_bb->SetInt("ROBOT_ID", robotID);
			private_bb->SetString("center", "point");
			private_bb->SetDouble("centerx", ballPos.x);
			private_bb->SetDouble("centery", ballPos.y);
			private_bb->SetDouble("radius", 0.1);
			private_bb->SetDouble("faceTowardsPosx", goalPos.x);
			private_bb->SetDouble("faceTowardsPosy", goalPos.y);
			private_bb->SetDouble("w", 3.0);
		}
		rotateAroundPoint.Update();
	
		return Status::Running;
	}
	else {
		stoprobot(robotID);
		return Status::Success;
		
	}
}

} // rtt




