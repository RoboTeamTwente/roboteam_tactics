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
    chip = false;

    dontDribble = (HasBool("dribblerOff") && GetBool("dribblerOff"));
    passThreshold = 0.1;    // minimal dist of opp to pass line for pass to be possible

    ros::param::getCached("robot_output_target", robot_output_target);

    if (GetBool("unclaimPos")) {
    // unclaim position
        ros::param::set("robot" + std::to_string(robotID) + "/claimedPosX", -0.0);
        ros::param::set("robot" + std::to_string(robotID) + "/claimedPosY", -0.0);
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << ", Unclaiming pos because doing GetBall now");
    }

    

    // // RANDOM ANGLE SHOOTER (set deviation to 0 to not use this)
        // if ((GetString("aimAt")=="ourgoal" || GetString("aimAt")=="theirgoal") && GetBool("passOn")) {
        //     deviation = 0.30*(get_rand_int(2)*2-1);
        //     ROS_INFO_STREAM("GetBall: " << deviation);
        // } else {
            deviation = 0.0;
        // }

    if (GetBool("passToBestAttacker")) {
        initializeOpportunityFinder();
    }
}

void GetBall::initializeOpportunityFinder() {
    // load the relevant weights list on which our teammates' positions will be judged, and specify the 'target' (see opportunityFinder)
    opportunityFinder.Initialize("jelle.txt", robotID, "theirgoal", 0);

    // the following custom changes apply for when choosing a teammate to pass to.
    opportunityFinder.setWeight("distToTeammate", 0.0);  // not relevant for passing, only for positioning
    opportunityFinder.setWeight("distToSelf", 0.0);      // not relevant for passing, only for positioning
    opportunityFinder.setMin("distOppToBallTraj", passThreshold);  // be less strict on whether pass will fail
}

void GetBall::Terminate(bt::Node::Status s) {
    if (!hasTerminated) {
    // Because I got this annoying terminate message constantly, even if the skill had not initialized again
        hasTerminated = true;
        releaseBall();
        ROS_INFO_STREAM_NAMED("skills.GetBall", "Terminating for robot " << robotID << ", releasing the ball if I claimed it");
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
            ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " already claimed ball");
            ballClaimedByMe = true; 
            return true;
        } else if (robotClaimedBall == -1) {
            //ros::param::set("robotClaimedBall", robotID);
            ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " claimed ball");
            ballClaimedByMe = true;
            return true; // if no-one claimed the ball -> I claim the ball
        } else {
            ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " cant claim the ball because " << robotClaimedBall << " already did");
            return false;
        }
    } else { // if param had not been initialized, set it anyways
        //ros::param::set("robotClaimedBall", robotID);
        ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " claimed ball");
        ballClaimedByMe = true;
        return true;
    }
}

void GetBall::releaseBall() {
    if (GetBool("useBallClaiming") && claimBall()) {
    // only release ball if I actually claimed it
        ros::param::set("robotClaimedBall", -1);
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", robotID << " released ball");
        ballClaimedByMe = false;
    }
    return;
}

void GetBall::passBall(int id) {
    if (GetBool("useBallClaiming") && claimBall()) {
    // only pass the claim on the ball if I actually claimed it
        ros::param::set("robotClaimedBall", id);
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", robotID << " passed claim on the ball to robot " << id);
        ballClaimedByMe = false;
    }
    return;
}

double GetBall::computePassSpeed(double dist, double v2) {
    double a = 1.0; // friction constant. assumes velocity decreases linearly over time
    double v1 = sqrt(v2*v2 + a*2*dist); // computes necessary pass speed v1, such that ball has speed v2 when it arrives.
    return v1;
}

bt::Node::Status GetBall::Update (){
    if (GetBool("useBallClaiming") && !claimBall()) {
    // In this way, only 1 of our robots may perform GetBall at a time. Is that what we want? probably yes
        // ROS_WARN_STREAM_NAMED("skills.GetBall", "return RUNNING for robot "<< robotID << ", because ball was already claimed");
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

    // Different GetBall parameters for grsim than for real robots
    double successDist;
    double successAngle;
    // double getBallDist; deprecated
    double distAwayFromBall;
    double minDist;
    if (robot_output_target == "grsim") {
        successDist = 0.12;
        successAngle = 0.15;
        // getBallDist = 0.0;
        distAwayFromBall = 0.28;
        minDist = 0.06;
    } else if (robot_output_target == "serial") {
        successDist = 0.12; //0.12
        successAngle = 0.15; //0.15
        // getBallDist = 0.0;
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
    // if (HasDouble("getBallDist")) {
    //     getBallDist=GetDouble("getBallDist");
    // }
    if (HasDouble("successDist")) {
        successDist=GetDouble("successDist");
    }


    // Check whether I should shoot at goal
    double viewOfGoal = opportunityFinder.calcViewOfGoal(ballPos, world);
    bool canSeeGoal = viewOfGoal >= 0.2; 
    bool shootAtGoal = GetBool("passToBestAttacker") && canSeeGoal
    		&& !(HasBool("dontShootAtGoal") && GetBool("dontShootAtGoal"));

    // If we should pass on to the best available attacker, choose this robot using opportunityfinder, based on the weightlist chosen in the initialization
    if (!choseRobotToPassTo && GetBool("passToBestAttacker") && posDiff.length() < 0.6 && !shootAtGoal) {
        BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
        bestID = bestTeammate.id;
        bestPos = bestTeammate.pos;
        choseRobotToPassTo = true;
        ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << ", passToRobot rosparam set to: " << bestID);
    }
    

	/* If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball */
	double targetAngle;
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
        } else {
        // aim at best teammate to pass to.
            targetAngle = (bestPos - ballPos).angle();
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
    // POSSIBLE IMPROVEMENT: DO SOMETHING ABOUT THE 'WIGGLE'
    double ballDist = minDist + (distAwayFromBall - minDist) / (0.5*M_PI) * fabs(angleDiff);
    if (ballDist > distAwayFromBall) {
        ballDist = distAwayFromBall;
    }
    Vector2 targetPos;
    if (fabs(angleDiff)>successAngle) {
        targetPos = ballPos + Vector2(-ballDist,0).rotate( posDiff.angle() + signum(angleDiff) * acos(minDist / ballDist) );
        private_bb->SetBool("dribbler", posDiff.length()<0.2 && !dontDribble && fabs(angleDiff)<M_PI/3);
    } else {
        targetPos = ballPos + Vector2(-ballDist, 0.0).rotate(targetAngle);
        private_bb->SetBool("dribbler", !dontDribble);
    }
    
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
            if (choseRobotToPassTo && ballCloseFrameCount == 0) {
            // If I chose best teammate before, check best robot once more (at the first success count)

                bool checkIfReady = GetBool("checkIfReady"); // if true, I will check if my teammate is ready to receive the ball, before passing.

                // If I couldn't find a suitable player before, try again one last time
                if (bestID == -1) {
                    BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
                    if (bestTeammate.id != -1) {
                        bestID = bestTeammate.id;
                        bestPos = bestTeammate.pos;
                    }
                } 

                // If again I didnt find a suitable player, or the passline to the chosen robot is blocked now...
                // ... -> go through some alternatives. If theres no alternative, chip at goal.
                if (bestID == -1 || opportunityFinder.calcDistOppToBallTraj(bestPos, world) < passThreshold) { 
                    // pass line is crossed by opponent -> chip possible?
                    double chipDist = 1.0;
                    Vector2 passLine = bestPos - ballPos;
                    Vector2 chipBallPos = ballPos + passLine.stretchToLength(chipDist); // ball pos after chip landed
                    roboteam_msgs::World chipWorld = world; // alternative world where the ball lay on the position after the chip landed
                    chipWorld.ball.pos.x = float(chipBallPos.x);
                    chipWorld.ball.pos.y = float(chipBallPos.y);
                    if (bestID == -1 || passLine.length() < chipDist || opportunityFinder.calcDistOppToBallTraj(bestPos, chipWorld) < passThreshold) {
                    // chip not possible -> softpass to someone else?
                        // First specify what leads to a soft pass being possible
                        opportunityFinder.setMin("distToOpp", 1.0); // if opponent will be closer than this value, soft pass wont be viable
                        opportunityFinder.setMax("distToOpp", 2.0); // make sure max value is still higher than newly chosen min value
                        // Find best teammate for a soft pass
                        BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
                        initializeOpportunityFinder(); // reset parameters of our opportunity finder for future usage.
                        if (bestTeammate.id == -1) {
                        // found no (claimed) pos to which a soft pass would be smart -> consider direct pass
                            BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(true, true, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
                            if (bestTeammate.id == -1) {
                            // found no good direct pass option -> direct chip to anyone? WIP. for now: chip towards goal
                                chip = true;
                                bestID = -1; // this leads to shootAtGoal in next iteration
                                checkIfReady = false;
                            } else {
                            // direct pass is possible
                                bestID = bestTeammate.id;
                                bestPos = bestTeammate.pos;
                                checkIfReady = false;
                            }
                        } else {
                        // soft pass is possible
                            bestID = bestTeammate.id;
                            bestPos = bestTeammate.pos;
                            checkIfReady = false;
                        }
                    } else {
                    // chip is possible -> keep this bestpos and chip!
                        chip = true;
                    }
                }

                if (checkIfReady && bestID != -1) {
                // If necessary, perform a last check to see if robot is ready to receive ball
                    bool readyToReceiveBall = false;
                    ros::param::get("robot" + std::to_string(bestID) + "/readyToReceiveBall", readyToReceiveBall);
                    if (!readyToReceiveBall) {
                    // if not ready: dont shoot yet
                        return Status::Running; 
                    } else {
                    // assigns ball claimage to new player that is passed to
                    // NOTE: this can be dangerous, so now only happens if the receiving robot actually communicates back.
                    // If the robot communicates, I can assume it will reset its own ball claimage if he failed to receive it.
                        passBall(bestID); 
                    }
                }

                ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball (possibly once more)
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << ", passToRobot param set to: " << bestID);
                // readyTimerStart = now();
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
                    // double passSpeed = targetDist*2.0;
                    // if (HasDouble("passingMult")) {
                    //     passSpeed = targetDist*GetDouble("passingMult");
                    // }
                    // if (passSpeed>6.5) {
                    //     passSpeed = 6.5;
                    // } else if (passSpeed<2.0) {
                    //     passSpeed = 2.0;
                    // }
                double passSpeed = 4.0;
                if (choseRobotToPassTo) {
                    passSpeed = computePassSpeed((bestPos - ballPos).length(), 2.0);
                }
                publishKickCommand(passSpeed);
                return Status::Success;
            } else {
            // Shooting hard
                releaseBall();
                publishKickCommand(6.5);
                return Status::Success;
            }
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
