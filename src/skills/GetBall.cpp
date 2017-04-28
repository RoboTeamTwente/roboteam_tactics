#include "ros/ros.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"

#include <cmath>
#include <vector>
#include <string>

#define RTT_CURRENT_DEBUG_TAG GetBall

namespace rtt {

RTT_REGISTER_SKILL(GetBall);

GetBall::GetBall(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {
    hasBall = whichRobotHasBall();

            std::string robot_output_target = "";
            ros::param::getCached("robot_output_target", robot_output_target);
            if (robot_output_target == "grsim") {
                distanceFromBallWhenDribbling = 0.105;
            } else if (robot_output_target == "serial") {
                distanceFromBallWhenDribbling = 0.08;
            } else {
                distanceFromBallWhenDribbling = 0.105;
            }
}

int GetBall::whichRobotHasBall() {
    auto holder = getBallHolder();
    if (!holder) {
        return -1;
    }
    our_team = holder->second;
    return holder->first.id;
}

void GetBall::publishStopCommand() {
	roboteam_msgs::RobotCommand command;
	command.id = robotID;
	command.x_vel = 0.0;
	command.y_vel = 0.0;
	command.w = 0.0;
	command.dribbler = false;

	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}

void GetBall::publishKickCommand(){

    double kicker_vel=5.0;
    if(HasDouble("kickerVel")){
        kicker_vel=GetDouble("kickerVel");
    }
    
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.kicker = GetBool("passOn");
    command.kicker_forced = GetBool("passOn");
    command.kicker_vel = GetBool("passOn") ? kicker_vel : 0;

    command.x_vel = 0;
    command.y_vel = 0;
    if (GetBool("passOn")) {
        command.dribbler = false;
    } else {
        command.dribbler = true;
    }
    

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);  
}


void GetBall::Initialize() {
    ballCloseFrameCount = 0;
}

bt::Node::Status GetBall::Update (){

    if(finalStage){
        if(countFinalMessages < 10){
            publishKickCommand();
            countFinalMessages=countFinalMessages+1;
            return Status::Running;
        }
        else {
            publishStopCommand();
            return Status::Success;
        }
    }

	roboteam_msgs::World world = LastWorld::get();
	robotID = blackboard->GetInt("ROBOT_ID");
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	}


	// Wait for the first world message
	while (world.us.size() == 0) {
		return Status::Running;
	}


	// Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    roboteam_msgs::WorldRobot robot;
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN_STREAM("GetBall: robot with this ID not found, ID: " << robotID);
    }  

	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	Vector2 ballPos(ball.pos);
	Vector2 ballVel(ball.vel);
	Vector2 robotPos(robot.pos);
	Vector2 robotVel(robot.vel);
	Vector2 targetPos;
	double targetAngle;

	// If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball
    // if (HasInt("aimAtRobot")) {
        // bool ourTeam = true;
        // if (HasBool("ourTeam")) {
            // if (GetBool("ourTeam")) {
                // ourTeam = true;
            // } else {
                // ourTeam = false;
            // }
        // }
        // targetAngle = GetTargetAngle(ballPos, "robot", GetInt("aimAtRobot"), ourTeam); // in roboteam_tactics/utils/utils.cpp
    // } 
    if (HasString("aimAt")) {
		targetAngle = GetTargetAngle(ballPos, GetString("aimAt"), GetInt("aimAtRobot"), GetBool("ourTeam")); // in roboteam_tactics/utils/utils.cpp
	} else {
		if (HasDouble("targetAngle")) {
			targetAngle = GetDouble("targetAngle");
		} else {
			Vector2 posdiff = ballPos - robotPos;
			targetAngle = posdiff.angle();
		}
	}
	targetAngle = cleanAngle(targetAngle);

    ROS_INFO_STREAM(robotID << " getball targetAngle: " << targetAngle << " robot angle: " << robot.angle);

	// Limit the difference between the targetAngle and the direction we're driving towards to 90 degrees so we don't hit the ball
	// This is no problem, because the direction we're driving towards slowly converges to the targetAngle as we drive towards the 
	// target position. It's hard to explain without drawing, for questions ask Jim :)
    double angleDiff = (targetAngle - (ballPos - robotPos).angle());
    // ROS_INFO_STREAM("angleDiff: " << angleDiff);
	angleDiff = cleanAngle(angleDiff);
    double intermediateAngle;
	if (angleDiff > 0.3*M_PI) {
		intermediateAngle = (ballPos - robotPos).angle() + 0.3*M_PI;
	} else if (angleDiff < -0.3*M_PI) {
		intermediateAngle = (ballPos - robotPos).angle() - 0.3*M_PI;
	} else {
        intermediateAngle = targetAngle;
    }
	

	// Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
	// at a distance of 30 cm of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
	double posDiff = (ballPos - robotPos).length();
    double getBallDist;
    if (HasDouble("getBallDist")) {
        getBallDist = GetDouble("getBallDist");
    } else {
        getBallDist = 0.06;
    }

    std::string robot_output_target = "";
    ros::param::getCached("robot_output_target", robot_output_target);
    double successDist;
    double successAngle;
    if (robot_output_target == "grsim") {
        successDist = 0.13;
        successAngle = 0.3;
    } else if (robot_output_target == "serial") {
        successDist = 0.11;
        if (HasDouble("successAngle")) {
            successAngle = GetDouble("successAngle");
        } else {
            successAngle = 0.3;
        }
    }
    if (HasDouble("successDist")) {
         successDist=GetDouble("successDist");
    }


    bool dribbler = false;
	if (posDiff > 0.5 || fabs(angleDiff) > successAngle) {
		targetPos = ballPos + Vector2(0.4, 0.0).rotate(intermediateAngle + M_PI);
	} else {
		// dribbler = true;
        private_bb->SetBool("dribbler", true);
        //private_bb->SetDouble("pGainPosition", 1.0);
		targetPos = ballPos + Vector2(getBallDist, 0.0).rotate(intermediateAngle + M_PI); // For arduinobot: 0.06
	}


    

    // targetAngle = (ballPos - robotPos).angle();
    // ROS_INFO_STREAM("posError: " << (ballPos - robotPos).length() << " angleError: " << cleanAngle(targetAngle - robot.angle));
    // ROS_INFO_STREAM("successDist: "<<successDist<< ", successAngle: "<<successAngle);
    double rotDiff = cleanAngle((ballPos - robotPos).angle() - robot.angle);
    double angleError = cleanAngle(robot.angle - targetAngle);


    // ROS_INFO_STREAM("dist: " << (ballPos - robotPos).length() << " angle: " << angleError);
	if ((ballPos - robotPos).length() < successDist && fabs(angleError) < successAngle) {
        // ROS_INFO_STREAM("we're there!");

        // Ideally we want to use the kick skill here, but it is the question whether that is fast enough to respond
        // in the situation when the ball is rolling and we are catching up
// =======
// 	if (stat == Status::Success 
//             && fabs(targetAngle - robot.angle) < 0.2*M_PI
//             // Only send succes if either:
//             //  - The ball was close for 8 or more frames
//             //  - The ball must be kicked as soon as there's a chance of kicking it
//             && (ballCloseFrameCount >= 3 || GetBool("passOn"))) {

// 		// Ideally we want to use the kick skill here, but it is the question whether that is fast enough to respond
// 		// in the situation when the ball is rolling and we are catching up
// >>>>>>> 1706a8dc695f3b77e4f8c4490b6ecd2471c55c58
        // double kicker_vel=5.0;
        // if(HasDouble("kickerVel")){
        //     kicker_vel=GetDouble("kickerVel");
        // }
        
        // roboteam_msgs::RobotCommand command;
        // command.id = robotID;
        // command.kicker = GetBool("passOn");
        // command.kicker_forced = GetBool("passOn");
        // command.kicker_vel = GetBool("passOn") ? kicker_vel : 0;

        // command.x_vel = 0;
        // command.y_vel = 0;
        // if (GetBool("passOn")) {
        //     command.dribbler = false;
        // } else {
        //     command.dribbler = true;
        // }
        

        // auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        // pub.publish(command);   
        // pub.publish(command); 

        if (ballCloseFrameCount < 5) {
            // ROS_INFO_STREAM("ballCounter " << ballCloseFrameCount);
            ballCloseFrameCount++;
            return Status::Running;
        } else {
            finalStage=true;
            publishKickCommand();
        }
        
        // return Status::Running;
    } else {
        ballCloseFrameCount = 0;
    }

    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", targetPos.x);
    private_bb->SetDouble("yGoal", targetPos.y);
    
    private_bb->SetDouble("angleGoal", targetAngle);
    private_bb->SetBool("avoidRobots", true);
    // private_bb->SetBool("dribbler", dribbler);
    private_bb->SetString("whichBot", GetString("whichBot"));
    
    // @HACK for robot testing purposes
    if (HasDouble("minSpeed")) {
    	private_bb->SetDouble("minSpeed", GetDouble("minSpeed"));
    }
    if (HasDouble("maxSpeed")) {
    	private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
    }
    if (HasDouble("pGainRotation")) {
    	private_bb->SetDouble("pGainRotation", GetDouble("pGainRotation"));
    }
    if (HasDouble("pGainPosition")) {
    	private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
    }
    
    if (HasString("stayOnSide")) {
        private_bb->SetString("stayOnSide", GetString("stayOnSide"));
    }
    if (HasDouble("avoidRobotsGain")) {
        private_bb->SetDouble("avoidRobotsGain", GetDouble("avoidRobotsGain"));
    }
    if (HasBool("forceAngle")) {
        private_bb->SetBool("forceAngle", GetBool("forceAngle"));
    }
    if (HasBool("smoothDriving")) {
        private_bb->SetBool("smoothDriving", GetBool("smoothDriving"));
        private_bb->SetDouble("smoothingNumber", GetDouble("smoothingNumber"));
    }

    /*
    if (HasDouble("successDist")) {
         private_bb->SetDouble("successDist", GetDouble("successDist"));
    }
    */
    boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
    roboteam_msgs::RobotCommand command;
    if (commandPtr) {
    	command = *commandPtr;
    } else {
    	ROS_WARN("GoToPos returned an empty command message! Maybe we are already there :O");
    }

    // TODO: Commented this out because it was giving problems. Hopefully we can
    // activate it at some point.
    // Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(1.0);
    // Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
    // if (newVelCommand.length() > 4.0) {
    //   newVelCommand.scale(4.0 / newVelCommand.length());
    // }
    // command.x_vel = newVelCommand.x;
    // command.y_vel = newVelCommand.y;

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);	

	return Status::Running;
}

} // rtt
