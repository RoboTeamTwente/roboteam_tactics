#include "ros/ros.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/conditions/CanClaimBall.h"
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
#include <boost/optional.hpp>

#define RTT_CURRENT_DEBUG_TAG GetBall

namespace rtt {

RTT_REGISTER_SKILL(GetBall);

GetBall::GetBall(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {
    std::string robot_output_target = "";
    ros::param::getCached("robot_output_target", robot_output_target);
    if (robot_output_target == "grsim") {
        distanceFromBallWhenDribbling = 0.105;
    } else if (robot_output_target == "serial") {
        distanceFromBallWhenDribbling = 0.08;
    } else {
        distanceFromBallWhenDribbling = 0.105;
    }
    choseRobotToPassTo = false;
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

    double kicker_vel = 5.0;
    if (HasDouble("kickerVel")) {
        kicker_vel = GetDouble("kickerVel");
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

bool GetBall::canClaimBall() {
    int robotClaimedBall;

    if (ros::param::has("robotClaimedBall")) {
        ros::param::getCached("robotClaimedBall", robotClaimedBall);

        if (robotClaimedBall == robotID) {
            return true;
        } else if (robotClaimedBall == -1) {
            ros::param::set("robotClaimedBall", robotID);
            return true;
        } else {
            ros::param::set("robotClaimedBall", -1);
            return false;
        }
    } else {
        ros::param::set("robotClaimedBall", robotID);
        return true;
    }
}

void GetBall::releaseBall() {
    ros::param::set("robotClaimedBall", -1);
    return;
}


void GetBall::Initialize() {
    ballCloseFrameCount = 0;
    finalStage = false;
    countFinalMessages = 0;

    
}


bt::Node::Status GetBall::Update (){

	roboteam_msgs::World world = LastWorld::get();
	robotID = blackboard->GetInt("ROBOT_ID");
    // if (!canClaimBall()) {return Status::Failure;}

    if (finalStage){
        if(countFinalMessages < 10){
            publishKickCommand();
            countFinalMessages=countFinalMessages+1;
            return Status::Running;
        }
        else {
            ROS_INFO_STREAM("GetBall Success robot " << robotID);
            //publishStopCommand();
            releaseBall();
            // if (GetBool("passToBestAttacker") && !choseRobotToPassTo && !shootAtGoal) {
            //     return Status::Failure;
            // }
            return Status::Success;
        }
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
        return Status::Failure;
    }  

	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	Vector2 ballPos(ball.pos);
	Vector2 ballVel(ball.vel);
	Vector2 robotPos(robot.pos);
	Vector2 robotVel(robot.vel);
	Vector2 targetPos;
	double targetAngle;
    Vector2 posDiff = ballPos - robotPos; 

    double viewOfGoal = opportunityFinder.calcViewOfGoal(robotPos, world);
    bool canSeeGoal = viewOfGoal >= 0.2; 
    bool shootAtGoal = GetBool("passToBestAttacker") && canSeeGoal && !(HasBool("dontShootAtGoal") && GetBool("dontShootAtGoal"));




    // If we should pass on to the best available attacker, we should find which one has the highest score
    if (posDiff.length() < 0.6 && GetBool("passToBestAttacker") && !choseRobotToPassTo && !shootAtGoal) {
        double maxScore = -std::numeric_limits<double>::max();

        for (size_t i = 0; i < (world.us.size()); i++) {

            std::string paramName = "robot" + std::to_string(world.us.at(i).id) + "/readyToReceiveBall";
            bool readyToReceiveBall = false;
            ros::param::getCached(paramName, readyToReceiveBall);

            if (world.us.at(i).id != (unsigned int) robotID && readyToReceiveBall) {
                opportunityFinder.Initialize("spits.txt", world.us.at(i).id, "theirgoal", 0);
                double score = opportunityFinder.computeScore(Vector2(world.us.at(i).pos));
                ROS_INFO_STREAM("evaluating: " << world.us.at(i).id << " score: " << score);
                if (score > maxScore) {
                    maxScore = score;
                    maxScoreID = world.us.at(i).id;
                }
            }
        }
        
        if (maxScore > -std::numeric_limits<double>::max()) {
            choseRobotToPassTo = true;
            ROS_INFO_STREAM("passing towards robot: " << maxScoreID);
        } else {
            SetString("aimAt", "theirgoal");
        }
        
    }
    

	// If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball
    if (GetBool("passToBestAttacker") && !choseRobotToPassTo && !shootAtGoal) {
        targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false); 
    } else if (choseRobotToPassTo) {
        targetAngle = GetTargetAngle(ballPos, "robot", maxScoreID, true);
    } else if (HasString("aimAt")) {
		targetAngle = GetTargetAngle(ballPos, GetString("aimAt"), GetInt("aimAtRobot"), GetBool("ourTeam")); // in roboteam_tactics/utils/utils.cpp
	} else if (HasDouble("targetAngle")) {
        targetAngle = GetDouble("targetAngle");
    } else if (shootAtGoal) {
        targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false);
    } else {
        targetAngle = posDiff.angle();
    }

    if (GetBool("aimAwayFromTarget")) {
        targetAngle = targetAngle + M_PI;
    }

	targetAngle = cleanAngle(targetAngle);


	// Limit the difference between the targetAngle and the direction we're driving towards to 90 degrees so we don't hit the ball
	// This is no problem, because the direction we're driving towards slowly converges to the targetAngle as we drive towards the 
	// target position. It's hard to explain without drawing, for questions ask Jim :)
    double angleDiff = (targetAngle - (ballPos - robotPos).angle());
	angleDiff = cleanAngle(angleDiff);
    double intermediateAngle;
	if (angleDiff > 0.2*M_PI) { // 0.1*M_PI for real-life robots!!
		intermediateAngle = (ballPos - robotPos).angle() + 0.3*M_PI;
	} else if (angleDiff < -0.2*M_PI) { // 0.1*M_PI for real-life robots!!
		intermediateAngle = (ballPos - robotPos).angle() - 0.3*M_PI;
	} else {
        intermediateAngle = targetAngle;
    }
	
    
	// Different GetBall parameters for grsim than for real robots
    std::string robot_output_target = "";
    ros::param::getCached("robot_output_target", robot_output_target);
    double successDist;
    double successAngle;
    double getBallDist;
    double distAwayFromBall;
    if (robot_output_target == "grsim") {
        successDist = 0.11;
        successAngle = 0.2;
        getBallDist = 0.00 ;
        distAwayFromBall = 0.2;;
    } else if (robot_output_target == "serial") {
        successDist = 0.11;
        successAngle = 0.15;
        getBallDist = 0.03;
        distAwayFromBall = 0.2;
    }
   
    if (HasDouble("getBallDist")){
        getBallDist=GetDouble("getBallDist");
    }
    // Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
    // at a distance of "distAwayFromBall" of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
	if (posDiff.length() > (distAwayFromBall + 0.3) || fabs(angleDiff) > (successAngle)) { // TUNE THIS STUFF FOR FINAL ROBOT
		targetPos = ballPos + Vector2(distAwayFromBall, 0.0).rotate(cleanAngle(intermediateAngle + M_PI));
	} else {
        private_bb->SetBool("dribbler", true);
        // if (robot_output_target == "serial") {
        //     private_bb->SetDouble("maxSpeed", 0.6);
        // }
		targetPos = ballPos + Vector2(getBallDist, 0.0).rotate(cleanAngle(intermediateAngle + M_PI)); // For arduinobot: 0.06        
	}
    

    // Return Success if we've been close to the ball for a certain number of frames
    double angleError = cleanAngle(robot.angle - targetAngle);
	if ((ballPos - robotPos).length() < successDist && fabs(angleError) < successAngle) {
        int ballCloseFrameCountTo = 20;
        if(HasInt("ballCloseFrameCount")){
            ballCloseFrameCountTo=GetInt("ballCloseFrameCount");
        }

        // if (GetBool("passToBestAttacker") && !choseRobotToPassTo && !shootAtGoal) {
        //     return Status::Running;
        // }
        
        if (ballCloseFrameCount < ballCloseFrameCountTo) {
            ballCloseFrameCount++;
            return Status::Running;
        } else {
            finalStage = true;
            publishKickCommand();
        }
    } else {
        ballCloseFrameCount = 0;
    }


    // Set the blackboard for GoToPos
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", targetPos.x);
    private_bb->SetDouble("yGoal", targetPos.y);
    private_bb->SetDouble("angleGoal", targetAngle);
    private_bb->SetBool("avoidRobots", false);
    if (HasBool("enterDefenseAreas")) {
        private_bb->SetBool("enterDefenseAreas", GetBool("enterDefenseAreas"));
    } 
    
    if (HasDouble("pGainPosition")) {
        private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
    } 
    if (HasDouble("pGainRotation")) {
        private_bb->SetDouble("pGainRotation", GetDouble("pGainRotation"));
    }

    // Get the velocity command from GoToPos
    boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
    roboteam_msgs::RobotCommand command;
    if (commandPtr) {
    	command = *commandPtr;
    }


    // Optional feature after testing: match the ball velocity for easy ball interception
    // if (HasBool("matchBallVel") && GetBool("matchBallVel")) {
    //     Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(1.0);
    //     Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
    //     if (newVelCommand.length() > 4.0) {
    //       newVelCommand.scale(4.0 / newVelCommand.length());
    //     }
    //     command.x_vel = newVelCommand.x;
    //     command.y_vel = newVelCommand.y;    
    // }
    
    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);	

	return Status::Running;
}

} // rtt
