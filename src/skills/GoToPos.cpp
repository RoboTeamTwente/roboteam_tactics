#include <string>

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

GoToPos::GoToPos(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {
        
	pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
}

void GoToPos::Initialize(int robotIDInput) {
	// robotID = robotIDInput;
}

void GoToPos::UpdateArgs (double xGoalInput, double yGoalInput, double wGoalInput) {
    // xGoal = xGoalInput;
	// yGoal = yGoalInput;
	// wGoal = wGoalInput;
}

bt::Node::Status GoToPos::Update (){
	roboteam_msgs::World world = LastWorld::get();

    double xGoal = GetDouble("xGoal");
    double yGoal = GetDouble("yGoal");
    double wGoal = GetDouble("wGoal");
    int robotID = blackboard->GetInt("ROBOT_ID");

    // std::cout << "I am: " << getPrefixedId("test") << "\n";
    // std::cout << "Going to: " << xGoal << " " << yGoal << "\n";
	
	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update 
	// is called before the first world state update
	if (world.robots_yellow.size() == 0) {
		ROS_INFO("No information about the world state :(");
		return Status::Running;
	}

	roboteam_msgs::WorldBall ball = world.ball;
	roboteam_msgs::WorldRobot robot = world.robots_yellow.at(robotID);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	double wCurrent = robot.w;

	// Proportional position controller
	roboteam_utils::Vector2 requiredSpeed;
    double pGain=3.0;
    double maxSpeed=1.0;
	requiredSpeed.x=(xGoal-robotPos.x)*pGain;
	requiredSpeed.y=(yGoal-robotPos.y)*pGain;
	if (requiredSpeed.length() > maxSpeed){
		requiredSpeed.x=requiredSpeed.x/requiredSpeed.length()*maxSpeed;
		requiredSpeed.y=requiredSpeed.y/requiredSpeed.length()*maxSpeed;
	}

	// Proportional rotation controller
	double requiredRotSpeed;
    double pGainRot=3.0;
    double maxRotSpeed=3.0;
    double rotError=wGoal-wCurrent;
    if (rotError > M_PI){rotError=M_PI-rotError;}
    requiredRotSpeed=rotError*pGainRot;
    if (fabs(requiredRotSpeed) > maxRotSpeed) {
    	requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }

    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 robotFrameRequiredSpeed;  
    robotFrameRequiredSpeed.x=requiredSpeed.x*cos(-wCurrent)-requiredSpeed.y*sin(-wCurrent);
    robotFrameRequiredSpeed.y=requiredSpeed.x*sin(-wCurrent)+requiredSpeed.y*cos(-wCurrent);

    // Fill up the command message
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = robotFrameRequiredSpeed.x;
    command.y_vel = robotFrameRequiredSpeed.y;
    command.w_vel = requiredRotSpeed;

    // If finished, return success, otherwise keep sending commands
    roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(xGoal, yGoal);
    roboteam_utils::Vector2 posError = goalPos - robotPos;
    
    prevWorld = world;
    if (posError.length() < 0.01) {
    	// Stop the robot and send one final command
    	roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
        command.w_vel = 0.0;
        pub.publish(command);
        ros::spinOnce();
    	return Status::Success;
    } else {
    	pub.publish(command);
    	return Status::Running;
    }
};

} // rtt
