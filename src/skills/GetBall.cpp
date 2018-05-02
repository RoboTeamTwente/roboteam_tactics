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
    startTime = now();
}

void GetBall::Initialize() {
    robotID = blackboard->GetInt("ROBOT_ID");
	ROS_INFO_STREAM_NAMED("skills.GetBall", "Initialize for robot: " << robotID);

    ballCloseFrameCount = 0;
    choseRobotToPassTo = false;
    ballClaimedByMe = false;
    hasTerminated = false;
    // chip = false;
    // startTime = now();

    dontDribble = (HasBool("dribblerOff") && GetBool("dribblerOff"));
    passThreshold = 0.2;    // minimal dist of opp to pass line for pass to be possible

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
    opportunityFinder.setMin("distOppToBallTraj", passThreshold);  // how strict to be on whether pass will fail
}

void GetBall::Terminate(bt::Node::Status s) {
    if (!hasTerminated) { // Temporary hack, because terminate is not always called at the right moments (similar hack in terminate function)
        hasTerminated = true;
        ballCloseFrameCount = 0;
        choseRobotToPassTo = false;
        // chip = false;

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

void GetBall::publishKickCommand(double kickSpeed, bool chip){
    if (HasDouble("kickerVel")) {
        kickSpeed = GetDouble("kickerVel");
    }
    choseRobotToPassTo = false;
    
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    if (chip) {
        command.chipper = true;
        command.chipper_forced = true;
        if (robot_output_target == "grsim" && kickSpeed*1.3 > 5.0) {
            command.chipper_vel = 5.0;
        } else {
            command.chipper_vel = kickSpeed*1.3;
        }
        
    } else {
        command.kicker = true;
        command.kicker_forced = true;
        command.kicker_vel = kickSpeed;
    }
    
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
        ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " released ball");
        // also reset passToRobot rosparam
        ros::param::set("passToRobot", -1);
    }
    ballClaimedByMe = false;
    return;
}

void GetBall::passBall(int id, Vector2 pos, Vector2 ballPos, bool chip) {

    double passSpeed = 4.0;
    if (choseRobotToPassTo) {
        double passDist = (pos - ballPos).length();
        double maxPassSpeed = computePassSpeed(passDist, 2.0, false); // fastest pass that my teammate can receive
        // WIP: TEST THIS PART
        double arrivalTime = computeArrivalTime(pos, id);
        passSpeed = computePassSpeed(passDist, arrivalTime, true);
        if (passSpeed > maxPassSpeed) {
            passSpeed = maxPassSpeed;
        }
    }

    if (GetBool("useBallClaiming") && id != -1 && claimBall()) { // only pass the claim on the ball if I actually claimed it
        bool readyToReceiveBall = false;
        ros::param::get("robot" + std::to_string(id) + "/readyToReceiveBall", readyToReceiveBall);
        if (readyToReceiveBall) {
            // assigns ball claimage to new player that is passed to
            // NOTE: this can be dangerous, so now only happens if the receiving robot actually communicates back.
            // If the robot communicates, I can assume it will reset its own ball claimage if he failed to receive it.
            ros::param::set("robotClaimedBall", id);
            ROS_WARN_STREAM_NAMED("skills.GetBall", robotID << " passed claim on the ball to robot " << id);
        }
    }
    ballClaimedByMe = false;
    ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " passed towards robot " << bestID);
    publishKickCommand(passSpeed, chip);
    return;
}

Vector2 GetBall::computeBallInterception(Vector2 ballPos, Vector2 ballVel, Vector2 myPos) { //WIP: needs testing for correctness
    // this function calculates the closest position where I could intercept the ball ..
    // .. assuming I would get there at a certain constant velocity and the ball would not slow down
    // These assumptions are of course not completely accurate, but might work for this purpose

    // used parameters
    double vBot = 2.0; // assumed constant robot velocity

    // used variables
    double vBall = ballVel.length();    // absolute ball velocity
    if (vBall < 0.1) {  // avoid division by 0 and switching behavior due to noise
        return ballPos; // ball velocity is low so I'll just go for the current ball position
    }
    double vux = ballVel.x / vBall;     // x component of ball velocity unit vector
    double vuy = ballVel.y / vBall;     // y component of ball velocity unit vector
    Vector2 rb = ballPos - myPos;       // vector from robot to ball
    double rbx = rb.x;                      // x component
    double rby = rb.y;                      // y component

    // formulas
    double term1 = vBot*vBot*(rbx*rbx + rby*rby) - vBall*vBall*pow(rbx*vuy + rby*vux, 2);
    double term2 = (vBall*vBall - vBot*vBot);
    if (term1 < 0 || fabs(term2) < 0.001) { 
    // prevent sqrt of negative number and division by 0 -> in this case the closest point from me to the ball trajectory becomes by best option
    // although this probably means I won't be in time (IMPROVEMENT: maybe should just take some margin ahead of ball as best option here)
        Vector2 closestPoint = ballPos - rb.project2(ballVel);
        return closestPoint;
    }
    double L = -vBall*( sqrt(term1) + vBall*(rbx*vux + rby*vuy)) / term2;
    if (L < 0) { // L should not be negative (no idea if that's possible at this point?)
        return ballPos;
    }
    Vector2 interceptPos = ballPos + ballVel.scale(L / vBall);
    return interceptPos;

}

double GetBall::computePassSpeed(double dist, double input, bool imposeTime) {
    double a = 1.5; // friction constant. assumes velocity decreases linearly over time
    if (HasDouble("friction")) {
        a = GetDouble("friction");
    }

    if (imposeTime) {
        // compute necessary pass speed, such that the ball arrives after 'arrivalTime'
        double arrivalTime = input;
        double maxTime = sqrt(dist*2/a); // imposing a higher arrival time is not possible
        if (arrivalTime > maxTime) {
            arrivalTime = maxTime;
        }
        if (arrivalTime < 0.001) { // prevent division by 0
            return 100;
        } else {
            double v1 = dist/arrivalTime + a*arrivalTime/2;
            return v1;
        }
    } else {
        // compute necessary pass speed, such that ball has speed v2 when it arrives.
        double v2 = input;
        double v1 = sqrt(v2*v2 + a*2*dist);
        return v1;
    }
}

double GetBall::computeArrivalTime(Vector2 location, Vector2 botPos, Vector2 botVel) {
    //  this function assumes the robot will accelerate towards its top speed with a constant
    //  acceleration in the direction of the location, without decelerating when it gets there.

    // used parameters
    double acc = 1.5; // m/s², assumes constant acceleration
    if (HasDouble("acc")) {
        acc = GetDouble("acc");
    }
    double vMax = 2.0; // max speed in m/s

    // A certain distance combined with current robot velocity relates to a certain time it takes to arrive
    Vector2 botToLoc = location - botPos;
    double dist = botToLoc.length();
    if (dist > 0.001) { // prevent division by 0
        double v1 = botVel.dot(botToLoc.scale(1/dist)); // current velocity in location direction
        if (v1 < 0) {
            v1 = 0;     // calculation does not make sense if v1 is negative
        }
        double v2 = sqrt(v1*v1 + 2*acc*dist); // velocity it would accelerate to
        if (v1 > vMax-0.001) { // robot already at max velocity
            return (dist / vMax);
        } else if (v2 > vMax) { // robot will reach its max velocity before covering the distance
            double t1 = (vMax - v1)/acc;        // time while accelerating
            double dist1 = t1*(v1 + vMax)/2;    // distance covered while accelerating
            double t2 = (dist - dist1) / vMax ; // time while at max speed
            return (t1 + t2);
        } else { // robot will not reach max velocity
            return (2*dist / (v1+v2));
        }
    } else {
        return 0.0; // practically already there
    }
}

double GetBall::computeArrivalTime(Vector2 location, int id) {
    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(id);
    roboteam_msgs::WorldRobot robot;
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN_STREAM_NAMED("skills.GetBall", "Teammate with this ID not found, ID: " << id);
        return 0.0;
    }
    Vector2 botPos(robot.pos);
    Vector2 botVel(robot.vel);

    return computeArrivalTime(location, botPos, botVel);
}


PassOption GetBall::choosePassOption(int passID, Vector2 passPos, Vector2 ballPos, roboteam_msgs::World world, double passThreshold) {
    // initialize my pass option
    PassOption passOption;
    passOption.chip = false;

    // If I couldn't find a suitable player before, try again one last time
    if (passID == -1) {
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " couldn't find a suitable player before, try again one last time");
        BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
        if (bestTeammate.id != -1) {
            passID = bestTeammate.id;
            passPos = bestTeammate.pos;
        }
    } 

    // If again I didnt find a suitable player, or the passline to the chosen robot is blocked now...
    // ... -> go through some alternatives. If theres no alternative, chip at goal.
    if (passID == -1 || opportunityFinder.calcDistOppToBallTraj(passPos, world) < passThreshold) { 
    // pass line is crossed by opponent -> chip possible?
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " couldnt do planned pass anymore, checking for chip to " << passID);
        double maxChipDist = 2.0;
        double minChipDist = 0.5;
        Vector2 passLine = passPos - ballPos;
        if (passID == -1 || passLine.length() < minChipDist || opportunityFinder.calcDistOppToBallTraj(passPos, world, maxChipDist) < passThreshold) {
        // chip not possible -> softpass to someone else?
            ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " couldnt do planned chip, checking for soft pass to another bot");
            // First specify what leads to a soft pass being possible
            opportunityFinder.setMin("distToOpp", 1.0); // if opponent will be closer than this value, soft pass wont be viable
            opportunityFinder.setMax("distToOpp", 2.0); // make sure max value is still higher than newly chosen min value
            // Find best teammate for a soft pass
            BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
            initializeOpportunityFinder(); // reset parameters of our opportunity finder for future usage.
            if (bestTeammate.id == -1) {
            // found no (claimed) pos to which a soft pass would be smart -> consider direct pass
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " couldnt find a soft pass candidate, checking for direct pass to a bot");
                BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(true, true, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
                if (bestTeammate.id == -1) {
                // found no good direct pass option -> direct chip to anyone? WIP. for now: chip towards goal. Better is probably chip towards edge defense area
                    ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " couldnt find a direct pass candidate, so chipping towards goal");
                    passOption.chip = true;
                    passID = -1; // this leads to shooting at goal
                } else {
                // direct pass is possible
                    passID = bestTeammate.id;
                    passPos = bestTeammate.pos;
                    ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " found a direct pass option and chose robot " << passID);
                }
            } else {
            // soft pass is possible
                passID = bestTeammate.id;
                passPos = bestTeammate.pos;
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " found a soft pass option and chose robot " << passID);
            }
        } else {
        // chip is possible -> keep this bestpos and chip!
            passOption.chip = true;
            ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " will chip towards robot " << passID);
        }
    }
    passOption.id = passID;
    passOption.pos = passPos;
    return passOption;
}

bt::Node::Status GetBall::Update (){
    if (hasTerminated) { // Temporary hack, because terminate is not always called at the right moments
        Initialize();
    }


    if (GetBool("useBallClaiming") && time_difference_milliseconds(startTime, now()).count() > 300) {
        if (!claimBall()){
        // In this way, only 1 of our robots may perform GetBall at a time. Is that what we want? probably yes
        // ROS_WARN_STREAM_NAMED("skills.GetBall", "return RUNNING for robot "<< robotID << ", because ball was already claimed");
            startTime = now();
            return Status::Running; // failure currently leads to too many reinitializations, so wont use that for now...
        }
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
    ballPos = computeBallInterception(ballPos, ballVel, robotPos); // pretend that the ball is at the location where we could intercept it
    Vector2 posDiff = ballPos - robotPos;

    drawer.setColor(72, 0, 255);
    drawer.drawPoint("ballIntercept",ballPos);

    // Different GetBall parameters for grsim than for real robots
    double successDist;
    double successAngle;
    // double getBallDist; deprecated
    double distAwayFromBall;
    double minDist;
    if (robot_output_target == "grsim") {
        successDist = 0.12;
        successAngle = 0.10;
        // getBallDist = 0.0;
        distAwayFromBall = 0.28;
        minDist = 0.06;
    } else if (robot_output_target == "serial") {
        successDist = 0.12; //0.12
        successAngle = 0.10; //0.15
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
    if (!choseRobotToPassTo && GetBool("passToBestAttacker") && posDiff.length() < 1.5 && !shootAtGoal) {
        BestTeammate bestTeammate = opportunityFinder.chooseBestTeammate(false, false, GetBool("doNotPlayBackDefender"), GetBool("doNotPlayBackAttacker"));
        bestID = bestTeammate.id;
        bestPos = bestTeammate.pos;
        choseRobotToPassTo = true;
        ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball
        ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << ", (first time) passToRobot rosparam set to: " << bestID << ", posDiff: " << posDiff.length());
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
    // Nothing given, shoot straight
    else {
        targetAngle = posDiff.angle();
    }

    if (GetBool("aimAwayFromTarget")) {
        targetAngle = targetAngle + M_PI;
    }

	targetAngle = cleanAngle(targetAngle);
    double angleDiff = cleanAngle(targetAngle - posDiff.angle());

    // Jelle's getBall motion variation:
    // POSSIBLE IMPROVEMENT: TAKE FUTURE BALL, OR BALL VELOCITY INTO ACCOUNT
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
        int ballCloseFrameCountTo = 2;
        if(HasInt("ballCloseFrameCount")){
            ballCloseFrameCountTo = GetInt("ballCloseFrameCount");
        } else if (dontDribble) {
            ballCloseFrameCountTo = 2;
        }
        
        if (ballCloseFrameCount < ballCloseFrameCountTo) {
        // When I have not been close for long enough yet
            ballCloseFrameCount++;
            return Status::Running;
        } else {
        // I have been close for long enough!
            bool chip = false;
            if (choseRobotToPassTo) {
            // If I chose best teammate before, check best robot once more (at the first success count)
                // struct PassOption contains int id, Vector2 pos and bool chip
                PassOption passOption = choosePassOption(bestID, bestPos, ballPos, world, passThreshold);
                chip = passOption.chip;
                if (passOption.id != bestID) {
                    bestID = passOption.id;
                    return Status::Running; // for getting a new target angle
                }
                bestPos = passOption.pos;
                ros::param::set("passToRobot", bestID); // communicate that chosen robot will receive the ball (possibly once more)
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << ", passToRobot rosparam set to: " << bestID);
                if (bestID == -1) {
                    shootAtGoal = true;
                }
            }

            if (!GetBool("passOn")) {
            // if not shooting, im successful now
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " has ball so succeeded");
                return Status::Success;
            } 
            else if (!shootAtGoal && (choseRobotToPassTo || (GetString("aimAt")=="robot" && GetBool("ourTeam"))) ) {
            // Passing to robot
                passBall(bestID, bestPos, ballPos, chip);
                return Status::Success;
            } else {
            // Shooting hard
                ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " shooting");
                if (GetBool("passToBestAttacker")) {
                    ros::param::set("passToRobot", -1); // communicate that chosen robot will receive the ball (possibly once more)
                    ROS_DEBUG_STREAM_NAMED("skills.GetBall", "robot " << robotID << " reset passToRobot rosparam to -1");
                }
                publishKickCommand(6.5, false);
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
