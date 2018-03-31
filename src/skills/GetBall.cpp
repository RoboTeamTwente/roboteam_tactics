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
    ballClaimedByMe = false;
}

void GetBall::Initialize() {
    robotID = blackboard->GetInt("ROBOT_ID");
	ROS_INFO_STREAM_NAMED("skills.GetBall", "Initialize for robot: " << robotID);

    ballCloseFrameCount = 0;
    choseRobotToPassTo = false;
    ballClaimedByMe = false;
    hasTerminated = false;
    startCheckingForSuccess = false;

    ros::param::getCached("robot_output_target", robot_output_target);

    if (GetBool("unclaimPos")) {
    // unclaim position
        ros::param::set("robot" + std::to_string(robotID) + "/claimedPosX", -0.0);
        ros::param::set("robot" + std::to_string(robotID) + "/claimedPosY", -0.0);
        ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", Unclaiming pos because doing GetBall now");
    }

    dontDribble = (HasBool("dribblerOff") && GetBool("dribblerOff"));

    // // RANDOM ANGLE SHOOTER (set deviation to 0 to not use this)
        // if ((GetString("aimAt")=="ourgoal" || GetString("aimAt")=="theirgoal") && GetBool("passOn")) {
        //     deviation = 0.30*(get_rand_int(2)*2-1);
        //     ROS_INFO_STREAM("GetBall: " << deviation);
        // } else {
            deviation = 0.0;
        // }

    if (GetBool("passToBestAttacker")) {
	// IMPROVE: Should be a different weight list that does not take dist to teammate (and dist to self?) into account
	    opportunityFinder.Initialize("jellePass.txt", robotID, "theirgoal", 0);
	}
}

void GetBall::Terminate(bt::Node::Status s) {
    if (!hasTerminated) {
    // Because I got this annoying terminate message constantly, even if the skill had not initialized again
        hasTerminated = true;
        releaseBall();
        ROS_INFO_STREAM_NAMED("skills.GetBall", "Terminating for robot " << robotID << ", releasing the ball");
    }
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

void GetBall::publishKickCommand(double kickSpeed){
    if (HasDouble("kickerVel")) {
        kickSpeed = GetDouble("kickerVel");
    }
    
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.kicker = true;
    command.kicker_forced = true;
    command.kicker_vel = kickSpeed;
    // command.chipper = GetBool("chipOn");
    // command.chipper_forced = GetBool("chipOn");
    // command.chipper_vel = GetBool("chipOn") ? kickSpeed : 0;
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = false;

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);  
}

bool GetBall::claimBall() {
    if (ballClaimedByMe) { // prevents too many ros param checks
        return true;
    }

    int robotClaimedBall;
    if (ros::param::has("robotClaimedBall")) {
        ros::param::getCached("robotClaimedBall", robotClaimedBall);

        if (robotClaimedBall == robotID) {
            ballClaimedByMe = true; 
            return true;
        } else if (robotClaimedBall == -1) {
            ros::param::set("robotClaimedBall", robotID);
            ballClaimedByMe = true;
            return true; // if no-one claimed the ball -> I claim the ball
        } else {
            ROS_WARN_DEBUG_NAMED("skills.GetBall", robotID << "cant claim the ball because " << robotClaimedBall << " already did");
            return false;
        }
    } else { // if param had not been initialized, set it anyways
        ros::param::set("robotClaimedBall", robotID);
        ballClaimedByMe = true;
        return true;
    }
}

void GetBall::releaseBall() {
    if (GetBool("useBallClaiming") && claimBall()) {
    // only release ball if I actually claimed it
        ros::param::set("robotClaimedBall", -1);
        ROS_WARN_DEBUG_NAMED("skills.GetBall", robotID << "released ball");
    }
    return;
}

void GetBall::passBall(int id) {
    if (GetBool("useBallClaiming") && claimBall()) {
    // only pass the claim on the ball if I actually claimed it
        ros::param::set("robotClaimedBall", id);
        ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << "passed claim on the ball to robot " << id);
    }
    return;
}

bt::Node::Status GetBall::Update (){
    if (GetBool("useBallClaiming") && !claimBall()) {
    // In this way, only 1 of our robots may perform GetBall at a time. Is that what we want? probably yes
        ROS_WARN_STREAM_NAMED("skills.GetBall", "return RUNNING for robot "<< robotID << ", because ball was already claimed");
        // return Status::Failure;
        return Status::Running; // failure currently leads to too many reinitializations, so wont use that for now...
    }

	roboteam_msgs::World world = LastWorld::get();

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
        ROS_WARN_STREAM_NAMED("skills.GetBall", "Robot with this ID not found, ID: " << robotID);
        return Status::Failure;
    }

    // Store some info about the world state
    roboteam_msgs::WorldBall ball = world.ball;
    Vector2 ballPos(ball.pos);
    Vector2 ballVel(ball.vel);
    Vector2 robotPos(robot.pos);
    Vector2 robotVel(robot.vel);
    Vector2 posDiff = ballPos - robotPos;

    // If I shot the ball, make sure it worked out
    if (startCheckingForSuccess) {
        if (elapsedTime)
        return Status::Success;
    }

    // Different GetBall parameters for grsim than for real robots
    double successDist;
    double successAngle;
    double getBallDist;
    double distAwayFromBall;
    double minDist;
    if (robot_output_target == "grsim") {
        successDist = 0.12;
        successAngle = 0.15;
        getBallDist = 0.0 ;
        distAwayFromBall = 0.28;
        minDist = 0.06;
    } else if (robot_output_target == "serial") {
        successDist = 0.12; //0.12
        successAngle = 0.15; //0.15
        getBallDist = 0.0;
        distAwayFromBall = 0.28;
        minDist = 0.06;

        // extra strafe gain for goToPos
        if (HasDouble("strafeGain")) {
	        private_bb->SetDouble("strafeGain", GetDouble("strafeGain"));
	    } else {
	    	private_bb->SetDouble("strafeGain", 1.3);
	    }
    }
    // if (GetBool("beAggressive", false)) {
        //  successDist = 0.11 ;
        //     successAngle = 0.15; 
        //     getBallDist = 0.0;
        //     distAwayFromBall = 0.2;
        //     // private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
        // }
    if (HasDouble("distAwayFromBall")) {
        distAwayFromBall = GetDouble("distAwayFromBall");
    }
    if (HasDouble("minDist")) {
        minDist = GetDouble("minDist");
    }
    if (HasDouble("successAngle")) {
        successAngle = GetDouble("successAngle");
    }
    if (HasDouble("getBallDist")) {
        getBallDist=GetDouble("getBallDist");
    }
    if (HasDouble("successDist")) {
        successDist=GetDouble("successDist");
    }


    // Check whether I should shoot at goal
    double viewOfGoal = opportunityFinder.calcViewOfGoal(ballPos, world);
    bool canSeeGoal = viewOfGoal >= 0.2; 
    bool shootAtGoal = GetBool("passToBestAttacker") && canSeeGoal
    		&& !(HasBool("dontShootAtGoal") && GetBool("dontShootAtGoal"));

    // If we should pass on to the best available attacker, choose this robot based on the weightlist as specified in the initialization
    if (!choseRobotToPassTo && GetBool("passToBestAttacker") && posDiff.length() < 0.6 && !shootAtGoal) {
        BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"), 8.0);
        bestID = bestTeammate.id;
        bestPos = bestTeammate.pos;
        choseRobotToPassTo = true;
        ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball
        ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", passToRobot param set to: " << bestID);
    }
    

	/* If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball */
	double targetAngle;
    double targetDist = 2.0; // WIP (jelle): for passing harder if target is further away
    // If we chose to shoot at goal
    if (shootAtGoal) { //GetBool("passToBestAttacker") && !choseRobotToPassTo && shootAtGoal
        targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false); 
    }
    // If a robot was found to pass to
    else if (choseRobotToPassTo) { 
        if (bestID == -1) {
        // could not find teammate to pass to, so WHAT SHOULD WE DO HERE?? FOR NOW ILL SET IT TO SHOOTATGOAL
            targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false);
            shootAtGoal = true;
            ROS_WARN_STREAM_NAMED("skills.GetBall", "robot " << robotID << " could not find a free teammate, so will shoot at goal");
        } else {
        // aim at best teammate to pass to.
            targetAngle = (bestPos - ballPos).angle();
            targetDist = (bestPos - ballPos).length(); 
        }
    } 
    // If a specific location to aim at was given
    else if (HasString("aimAt")) {
		targetAngle = GetTargetAngle(ballPos - Vector2(0,deviation), GetString("aimAt"), GetInt("aimAtRobot"), GetBool("ourTeam")); // in roboteam_tactics/utils/utils.cpp
	}
    // If a specific angle was given to aim at
    else if (HasDouble("targetAngle")) {
        targetAngle = GetDouble("targetAngle");
    }
    // If robot has to shoot at the goal
    else if (shootAtGoal) {
        targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false);
    }
    // Nothing given, shoot straight
    else {
        targetAngle = posDiff.angle();
    }

    if (GetBool("aimAwayFromTarget")) {
        targetAngle = targetAngle + M_PI;
    }

	targetAngle = cleanAngle(targetAngle);
    double angleDiff = cleanAngle(targetAngle - posDiff.angle());

    // // Limit the difference between the targetAngle and the direction we're driving towards to 90 degrees so we don't hit the ball
        // // This is no problem, because the direction we're driving towards slowly converges to the targetAngle as we drive towards the 
        // // target position. It's hard to explain without drawing, for questions ask Jim :)
        // double intermediateAngle;
        // if (angleDiff > 0.1*M_PI) { // 0.1*M_PI for real-life robots!!
        //     intermediateAngle = posDiff.angle() + 0.3*M_PI;
        // } else if (angleDiff < -0.1*M_PI) { // 0.1*M_PI for real-life robots!!
        //     intermediateAngle = posDiff.angle() - 0.3*M_PI;
        // } else {
        //     intermediateAngle = targetAngle;
        // }
    	// ROS_INFO_STREAM_NAMED("skills.GetBall", "intermediateAngle: " << intermediateAngle);

        // bool matchBallVel = false;
        // if (fabs(angleDiff) > 0.5*M_PI) {
        //     matchBallVel = true;
        // }
        // double addBallSpeed = ballVel.length() * 0.2;
        // if (addBallSpeed > 1.7) {
        //     addBallSpeed = 1.7;
        // }
        // distAwayFromBall = distAwayFromBall + addBallSpeed;

    // Jelle's getBall motion variation:
    double ballDist = minDist + (distAwayFromBall-minDist) / (0.5*M_PI) * fabs(angleDiff);
    if (ballDist > distAwayFromBall) {
        ballDist = distAwayFromBall;
    }
    Vector2 targetPos;
    if (fabs(angleDiff)>successAngle) {
        targetPos = ballPos + Vector2(-ballDist,0).rotate( posDiff.angle() + signum(angleDiff) * acos(minDist / ballDist) );
        private_bb->SetBool("dribbler", posDiff.length()<0.2 && !dontDribble && fabs(angleDiff)<M_PI/3);
    } else {
        targetPos = ballPos + Vector2(-getBallDist, 0.0).rotate(targetAngle);
        private_bb->SetBool("dribbler", !dontDribble);
    }
    
    //    // Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
     //    // at a distance of "distAwayFromBall" of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
    	// if (posDiff.length() > (distAwayFromBall + 0.3) || fabs(angleDiff) > (successAngle)) { // TUNE THIS STUFF FOR FINAL ROBOT
    	// 	targetPos = ballPos + Vector2(distAwayFromBall, 0.0).rotate(cleanAngle(intermediateAngle + M_PI));
     //        private_bb->SetBool("dribbler", false);
     //        matchBallVel = true;
     //        if (posDiff.length() < (distAwayFromBall + 0.1)) {
     //            moreGain = true;
     //        }
    	// } else {
    	// 	targetPos = ballPos + Vector2(getBallDist, 0.0).rotate(cleanAngle(intermediateAngle + M_PI)); // For arduinobot: 0.06       
     //        private_bb->SetBool("dribbler", !dontDribble);
    	// }
    
    // Return Success if we've been close to the ball for a certain number of frames
    double angleError = cleanAngle(robot.angle - targetAngle);
	if ((ballPos - robotPos).length() < successDist && fabs(angleError) < successAngle && fabs(angleDiff) < successAngle) {
        // matchBallVel = false;
        int ballCloseFrameCountTo = 5;
        if(HasInt("ballCloseFrameCount")){
            ballCloseFrameCountTo = GetInt("ballCloseFrameCount");
        } else if (dontDribble) {
            ballCloseFrameCountTo = 2;
        }
        
        if (ballCloseFrameCount < ballCloseFrameCountTo) {
        // When I have not been close for long enough yet
            if (choseRobotToPassTo && ballCloseFrameCount == 2) {
            // If I chose best teammate before, check best robot once more (at the second success count)
            // This time use actual position of teammates in assessment, instead of possibly claimed position
                BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"), 8.0, true);
                if (bestTeammate.id != bestID) {
                    bestID = bestTeammate.id;
                    bestPos = bestTeammate.pos;
                    
                }
                ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball (once more)
                ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", passToRobot param set to: " << bestID);
                
            }
            ballCloseFrameCount++;
            return Status::Running;
        } else {
        // I have been close for long enough!
            if (!GetBool("passOn")) {
            // if not shooting, im successful now
                return Status::Success;
            } 
            else if (!shootAtGoal && (choseRobotToPassTo || (GetString("aimAt")=="robot" && GetBool("ourTeam"))) ) {
            // Passing to robot
                if (GetBool("checkIfReady") && bestID != -1) {
                // If necessary, perform a last check to see if robot is ready to receive ball
                    bool readyToReceiveBall = false;
                    ros::param::get("robot" + std::to_string(bestID) + "/readyToReceiveBall", readyToReceiveBall);
                    if (!readyToReceiveBall) {
                    // if not ready: dont shoot yet
                        return Status::Running;
                    }
                }
                // If ready, actually execute the pass
                double passingSpeed = targetDist*2.0;
                if (HasDouble("passingMult")) {
                    passingSpeed = targetDist*GetDouble("passingMult");
                }
                if (passingSpeed>5.5) {
                    passingSpeed = 5.5;
                } else if (passingSpeed<2.0) {
                    passingSpeed = 2.0;
                }
                passBall(bestID);
                publishKickCommand(passingSpeed);
            } else {
            // Shooting hard
                releaseBall();
                publishKickCommand(6.5);
            }
            startCheckingForSuccess = true;
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
    private_bb->SetBool("avoidRobots", true);
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

        // Optional feature after testing: match the ball velocity for easy ball interception
            // if (matchBallVel) {
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
    } else {
        publishStopCommand();
        return Status::Running;
    }
    
}

} // rtt
