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
    passToRobot = 0;
}

void GetBall::Initialize() {
    robotID = blackboard->GetInt("ROBOT_ID");
	ROS_INFO_STREAM_NAMED("skills.GetBall", "Initialize for robot: " << robotID);

    ballCloseFrameCount = 0;
    choseRobotToPassTo = false;
    bestBotClaimedPos = false;
    // ros::param::set("passToRobot", -1);
    // ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", passToRobot param reset to -1 because doing GetBall now");
    ros::param::getCached("robot_output_target", robot_output_target);
    // unclaim position
    ros::param::set("robot" + std::to_string(robotID) + "/claimedPosX", -0.0);
    ros::param::set("robot" + std::to_string(robotID) + "/claimedPosY", -0.0);
    ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", Unclaiming pos because doing GetBall now");

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
    command.kicker = GetBool("passOn");
    command.kicker_forced = GetBool("passOn");
    command.chipper = GetBool("chipOn");
    command.chipper_forced = GetBool("chipOn");
    command.chipper_vel = GetBool("chipOn") ? kickSpeed : 0;
    command.kicker_vel = GetBool("passOn") ? kickSpeed : 0;

    command.x_vel = 0;
    command.y_vel = 0;
   if (GetBool("passOn") || dontDribble) {
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
            return true; // if no-one claimed the ball -> I claim the ball
        } else {
            ros::param::set("robotClaimedBall", -1); // SHOULD THIS BE DONE LIKE THIS? RATHER USE TERMINATE FUNCTION FOR UNCLAIMING
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

bt::Node::Status GetBall::Update (){

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

	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	Vector2 ballPos(ball.pos);
	Vector2 ballVel(ball.vel);
	Vector2 robotPos(robot.pos);
	Vector2 robotVel(robot.vel);
    Vector2 posDiff = ballPos - robotPos; 


    // Check whether I should shoot at goal
    double viewOfGoal = opportunityFinder.calcViewOfGoal(robotPos, world);
    bool canSeeGoal = viewOfGoal >= 0.2; 
    bool shootAtGoal = GetBool("passToBestAttacker") && canSeeGoal
    		&& !(HasBool("dontShootAtGoal") && GetBool("dontShootAtGoal"));

    // If we should pass on to the best available attacker, we should find which one has the highest score.
    if (posDiff.length() < 0.6 && GetBool("passToBestAttacker") && !choseRobotToPassTo && !shootAtGoal) {
        // ROS_INFO_STREAM("robot: " << robotID << " checking passToBestAttacker options");
        double maxScore = 0;//-std::numeric_limits<double>::max();
        boost::optional<int> maxScoreID = boost::none;

        // For each robot in the field
        for (size_t i = 0; i < (world.us.size()); i++) {
            if (world.us.at(i).id != (unsigned int) robotID) {

                // first check if current robot claimed a position. If so, score this claimed position
                double botClaimedX;
                ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosX", botClaimedX);
                if( !(botClaimedX == 0.0 && std::signbit(botClaimedX)) ) { // if not -0.0, bot actually claimed a position
                    double botClaimedY;
                    ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosY", botClaimedY);
                    double score = opportunityFinder.computeScore(Vector2(botClaimedX,botClaimedY),world);


                    // when "doNotPlayBackDefense" is true, this checks if this robot is behind me. If so, his score is set to 0
                    if(HasBool("doNotPlayBackDefense") && GetBool("doNotPlayBackDefense")) {
                        if (botClaimedX < robotPos.x) {
                            score = 0;
                        }
                    }
                    // when "doNotPlayBackAttack" is true, this checks if this robot is behind me. If so, his score is set to 0
                    if(HasBool("doNotPlayBackAttack") && GetBool("doNotPlayBackAttack")) {
                        if (calcDistGoalToRobot(i, world) > 8.0) {
                            score = 0;
                        }
                    }


                    if (score > maxScore) {
                        maxScore = score;
                        maxScoreID = world.us.at(i).id;
                        bestClaimedPos = Vector2(botClaimedX,botClaimedY);
                        bestBotClaimedPos = true;
                    }
                    // world.us.at(i).pos.x = float(botClaimedX);
                    // world.us.at(i).pos.y = float(botClaimedY);



                } else { // if current robot did not claim a position, check its own position.
                    double score = opportunityFinder.computeScore(Vector2(world.us.at(i).pos),world);


                    // when "doNotPlayBackDefense" is true, it checks if this robot is behind me. If so, his score is set to 0
                    if(HasBool("doNotPlayBackDefense") && GetBool("doNotPlayBackDefense")) {
                        if (Vector2(world.us.at(i).pos).x < robotPos.x) {
                            score = 0;
                        }
                    }
                    // when "doNotPlayBackAttack" is true, it checks if this robot is further away from their goal then 8m. If so, his score is set to 0
                    if(HasBool("doNotPlayBackAttack") && GetBool("doNotPlayBackAttack")) {
                        std::cout << "GETBALL - DistGoalToRobot " << i << " = " << calcDistGoalToRobot(i, world) << std::endl;
                        if (calcDistGoalToRobot(i, world) > 8.0) {
                            std::cout << "GETBALL - Set score of robot " << i << " to zero " << std::endl;
                            score = 0;
                        }
                    }


                    if (score > maxScore) {
                        maxScore = score;
                        maxScoreID = world.us.at(i).id;
                        bestBotClaimedPos = false;
                    }
                }

                // std::string paramName = "robot" + std::to_string(world.us.at(i).id) + "/readyToReceiveBall";
                // bool readyToReceiveBall = false;
                // ros::param::getCached(paramName, readyToReceiveBall);

                // if (world.us.at(i).id != (unsigned int) robotID && readyToReceiveBall) {

                // if (readyToReceiveBall){
                    // ROS_INFO_STREAM("getball " << robotID << " robot " << world.us.at(i).id << " readyToReceiveBall");
                    // opportunityFinder.Initialize("spits.txt", world.us.at(i).id, "theirgoal", 0);
                // }
            }
        } // for every robot
        
        if (maxScoreID)  {//-std::numeric_limits<double>::max() || GetBool("dontShootAtGoal", false)) {
            choseRobotToPassTo = true;
            passToRobot = *maxScoreID;
            // ROS_INFO_STREAM("passing to robot: " << passToRobot << " with score: " << maxScore);
            ros::param::set("passToRobot", passToRobot); // communicate that chosen robot will receive the ball
            ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", passToRobot param set to: " << passToRobot <<" because GetBall chose this attacker with score: " << maxScore);
        // } else if (GetBool("dontShootAtGoal", false)) {
        //     ROS_WARN("GetBall found no robot to pass to, but is not allowed to shoot at goal");

        } else {
            ROS_WARN_NAMED("skills.GetBall", "Found no robot to pass to");
            return Status::Failure;
            // shootAtGoal = true;
        	// SetString("aimAt", "theirgoal");
        }
    }
    


	/* If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball */
	double targetAngle;
    double targetDist = 2.0; // WIP (jelle): for passing harder if target is further away
    // If no robot was found to pass the ball to
    if (GetBool("passToBestAttacker") && !choseRobotToPassTo && shootAtGoal) {
        targetAngle = GetTargetAngle(ballPos, "theirgoal", 0, false); 
    } 
    // If a robot was found to pass to
    else if (choseRobotToPassTo) { 
        if (bestBotClaimedPos) {
            targetAngle = (bestClaimedPos - ballPos).angle();
            targetDist = (bestClaimedPos - ballPos).length();
        } else {
            targetAngle = GetTargetAngle(ballPos, "robot", passToRobot, true);
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
            ballCloseFrameCount++;
            return Status::Running;
        } else {

            if (choseRobotToPassTo || (GetString("aimAt")=="robot" && GetBool("ourTeam")) ) {
                double passingSpeed = targetDist*2;
                if (HasDouble("passingMult")) {
                    passingSpeed = targetDist*GetDouble("passingMult");
                }
                if (passingSpeed>6.5) {
                    passingSpeed = 6.5;
                } else if (passingSpeed<2.0) {
                    passingSpeed = 2.0;
                }
                publishKickCommand(passingSpeed);
            } else {
                publishKickCommand(6.5);
            }
            releaseBall();
            return Status::Success;
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
