#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#define RTT_CURRENT_DEBUG_TAG GetBall

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , success(false)
        
        // Rest of the members
        , safetyMarginGoalAreas(0.2)
        , marginOutsideField(1.2)

        , robotType(RobotType::PROTO)

        , lastRobotTypeError(now())
        , lastPresetError(now())
        
        {
            succeeded = false;
            failure = false;
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);
        }

RobotType GoToPos::getRobotType() {
    int const ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    std::string const robotTypeKey = "robot" + std::to_string(ROBOT_ID) + "/robotType";

    if (ros::param::has(robotTypeKey)) {
        std::string robotType;
        ros::param::getCached("robot" + std::to_string(ROBOT_ID) + "/robotType", robotType);

        if (robotType == "arduino") {
            return RobotType::ARDUINO;
        } else if (robotType == "proto") {
            return RobotType::PROTO;
        } else if (robotType == "grsim") {
            return RobotType::GRSIM;
        } else if (robotType == "") {
            if (time_difference_milliseconds(lastRobotTypeError, now()).count() >= 1000) {
                ROS_INFO_STREAM("Empty value for found for param \"" 
                        << robotTypeKey
                        << "\" found. Defaulting to RobotTypes::PROTO.\n"
                        );
                lastRobotTypeError = now();
            }
        } else {
            if (time_difference_milliseconds(lastRobotTypeError, now()).count() >= 1000) {
                ROS_ERROR_STREAM("Unknown value found for param \"" 
                        << robotTypeKey 
                        << "\": \"" 
                        << robotType
                        << "\". Defaulting to RobotType::PROTO."
                        );
                lastRobotTypeError = now();
            }
        }
    } else {
        if (time_difference_milliseconds(lastRobotTypeError, now()).count() >= 1000) {
            ROS_INFO_STREAM("No value for found for param \"" << robotTypeKey << "\" found. Defaulting to RobotTypes::PROTO.\n");
            lastRobotTypeError = now();
        }
    }

    return RobotType::PROTO;
}

void GoToPos::setPresetControlParams(RobotType newRobotType) {
    if (newRobotType == RobotType::ARDUINO) {
        // ROS_INFO_STREAM("ARDUINO control parameters set!");
        pGainPosition = 1.0;
        pGainRotation = 2.0;
        minSpeedX = 0.7;
        minSpeedY = 1.0; 
        maxSpeed = 3.0; 
        minAngularVel = 5.0;
        maxAngularVel = 10.0;

        robotType = RobotType::ARDUINO;
    } else if (newRobotType == RobotType::PROTO) {
        // ROS_INFO_STREAM("PROTO control parameters set!");
        pGainPosition = 2.0;
        pGainRotation = 3.0;
        minSpeedX = 0.4;
        minSpeedY = 0.6;
        maxSpeed = 1.0;
        minAngularVel = 3.0;
        maxAngularVel = 10.0;

        robotType = RobotType::PROTO;
    } else if (newRobotType == RobotType::GRSIM) {
        // ROS_INFO_STREAM("GRSIM control parameters set!");
        pGainPosition = 2.0;
        pGainRotation = 4.0;
        minSpeedX = 0.0;
        minSpeedY = 0.0;
        maxSpeed = 1.5;
        minAngularVel = 0.0;
        maxAngularVel = 10.0;

        robotType = RobotType::GRSIM;
    } else {
        if (time_difference_milliseconds(lastPresetError, now()).count() >= 1000) {
            ROS_ERROR_STREAM("Could not set robot type of " 
                    << blackboard->GetInt("ROBOT_ID")
                    << ". Undefined type. Leaving the type on: "
                    << (int) robotType
                    << "\n"
                    );

            lastPresetError = now();
        }
    }
}

void GoToPos::setPresetControlParams() {
    setPresetControlParams(getRobotType());
}

void GoToPos::setPresetControlParams(
    	double pGainPosition,
    	double pGainRotation,
    	double maxAngularVel,
    	double minAngularVel,
    	double maxSpeed,
    	double minSpeedX,
    	double minSpeedY) {
	this->pGainPosition = pGainPosition;
	this->pGainRotation = pGainRotation;
	this->maxAngularVel = maxAngularVel;
	this->minAngularVel = minAngularVel;
	this->maxSpeed = maxSpeed;
	this->minSpeedX = minSpeedX;
	this->minSpeedY	= minSpeedY;
}

void GoToPos::setPGains(double position, double rotation) {
	pGainPosition = position;
	pGainRotation = rotation;
}

void GoToPos::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    if (GetBool("dribbler")) {
        command.dribbler = true;
    } else {
        command.dribbler = false;
    }

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Proportional position controller
Vector2 GoToPos::positionController(Vector2 myPos, Vector2 targetPos) {
    Vector2 posError = targetPos - myPos;

    // Make the controller less strong once we get very close, to make it more precise for short distances
    if (posError.length() < 0.2) {
        pGainPosition = 1.0;
    }

    Vector2 velTarget = posError*pGainPosition;
    
    if (velTarget.length() > maxSpeed) {
        velTarget = velTarget.scale(maxSpeed / velTarget.length());
    }

    return velTarget;
}


// Proportional rotation controller
double GoToPos::rotationController(double myAngle, double angleGoal, Vector2 posError) {

    bool forceAngle = false;
    if(HasBool("forceAngle") && GetBool("forceAngle")){
        forceAngle = true;
    }

    // if (posError.length() > 1.0 && !forceAngle) {
    //     angleGoal = posError.angle();
    // }

    // Proportional term
    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);

    // // Integral term
    // double updateRate;
    // ros::param::get("role_iterations_per_second", updateRate);
    // double timeStep = 1 / updateRate;
    // angleErrorIntegral += angleError * timeStep;
    
    // // Differential term (UNTESTED!)
    // angleErrorHistory[historyIndex] = angleError;
    // int amountOfHistories = 4;
    // historyIndex = (historyIndex + 1) % amountOfHistories;
    // double avgHistory = 0;
    // for (int i = 0; i < amountOfHistories; i++) {
    //     avgHistory += angleErrorHistory[i];
    // }
    // avgHistory /= ((double) amountOfHistories);
    // double angleErrorDiff = avgHistory * timeStep;

    // Maybe we later choose to add an integral and/or differential term to this controller if it proves to be necessary, the basic structure
    // for this is already there, although it is not yet well tested
    double angularVelTarget = angleError * pGainRotation;    

    // Limit the angular velocity target
    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }

    return angularVelTarget;
}


Vector2 GoToPos::limitAngleDiff(Vector2 vector1, Vector2 vector2, double maxAngleDiff) {
    double angleDiff = cleanAngle(vector1.angle() - vector2.angle());
    if (angleDiff > maxAngleDiff) {
        vector1 = vector2.scale(vector1.length() / vector2.length()).rotate(maxAngleDiff);
    } else if (angleDiff < -maxAngleDiff) {
        vector1 = vector2.scale(vector1.length() / vector2.length()).rotate(-maxAngleDiff);
    }
    return vector1;
}

// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them
Vector2 GoToPos::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos) {

    Vector2 forceVector;
    Vector2 posError = targetPos - myPos;

    Vector2 ahead = myPos + antenna;
    if ((ahead - otherRobotPos).length() < 0.2) {
        Vector2 force = ahead - otherRobotPos;
        force = force.scale(avoidRobotsGain / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = force;
    }

    Vector2 ahead2 = myPos + antenna.scale(0.5);
    if ((ahead2 - otherRobotPos).length() < 0.2) {
        Vector2 force = ahead2 - otherRobotPos;
        force = force.scale(avoidRobotsGain / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = forceVector + force;
    }

    if ((myPos - otherRobotPos).length() < 0.3) {
        Vector2 force = myPos - otherRobotPos;
        force = force.scale((avoidRobotsGain) / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = forceVector + force;
    }

    return forceVector;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
    roboteam_msgs::World world = LastWorld::get();

    Vector2 posError = targetPos - myPos;
    double lookingDistance = 0.5; // default
    if (lookingDistance > (posError.length()) + 0.0) {
        lookingDistance = posError.length() + 0.0;
    }
    
    Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
    antenna = antenna.scale(myVel.length()*1.5);

    drawer.drawLine("antenna", myPos, antenna);

    Vector2 sumOfForces;
    for (auto const currentRobot : world.us) {
        if (currentRobot.id != ROBOT_ID) {
            Vector2 otherRobotPos(currentRobot.pos);
            Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, antenna, targetPos);
            sumOfForces = sumOfForces + forceVector;
        }
    }

    for (size_t i = 0; i < world.them.size(); i++) {
        Vector2 otherRobotPos(world.them.at(i).pos);
        Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, antenna, targetPos);
        sumOfForces = sumOfForces + forceVector; 
    }

    return sumOfForces;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 GoToPos::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
    Vector2 posError = targetPos - myPos;

    if (ROBOT_ID != KEEPER_ID) {
        Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", myPos, 0.0);
        if (fabs(distToOurDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToOurDefenseArea) > 0) {
            if (sumOfForces.dot(distToOurDefenseArea.rotate(0.5*M_PI)) > 0) {
                sumOfForces = distToOurDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            } else {
                sumOfForces = distToOurDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            }
        }
    }

    Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", myPos, 0.0);
    if (fabs(distToTheirDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToTheirDefenseArea) > 0) {
        if (sumOfForces.dot(distToTheirDefenseArea.rotate(0.5*M_PI)) > 0) {
            sumOfForces = distToTheirDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        } else {
            sumOfForces = distToTheirDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        }
    }

    return sumOfForces;
}


// Vector2 GoToPos::avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces) {
//     Vector2 diff = ballPos - myPos;
//     double theta = fabs(cleanAngle(diff.angle() - sumOfForces.angle()));

//     if (theta < (0.5 * M_PI)) {
//         if (theta == 0) theta = 0.01;
//         double force = theta / (0.5 * M_PI);
//         Vector2 projectedBall = ballPos.project(myPos, myPos + sumOfForces);
//         Vector2 ballForce = projectedBall - ballPos;
//         sumOfForces = sumOfForces + ballForce * 5;
//     }

//     return sumOfForces;
// }


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field. "Too far" is specified by the class variable marginOutsideField
Vector2 GoToPos::checkTargetPos(Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    // We should not go outside the field close to the goal areas
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = 0.0; 
    }

    // If the target position is outside of the field + margins, then change the target position to the closest point within this margin
    if (GetBool("ignoreFieldBounds", false)) {
        if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
            xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
        }
        if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
            yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
        }
    }

    Vector2 newTargetPos(xGoal, yGoal);

    // If the current robot is not a keeper, we should take into account that it cannot enter the defense area
    if (ROBOT_ID != KEEPER_ID) {
        // TODO: Doesn't work if normalize_field (rosparam in mini.launch) is true. Disabled until further notice    

        // // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        // if (isWithinDefenseArea("our defense area", newTargetPos)) {
            // Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
            // newTargetPos = newTargetPos + distToOurDefenseArea;
        // }

        // // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        // if (isWithinDefenseArea("their defense area", newTargetPos)) {
            // Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);
            // newTargetPos = newTargetPos + distToTheirDefenseArea;
        // }
    }

    // Check if we have to stay on our side of the field
    if (HasString("stayOnSide")) {
        auto side = GetString("stayOnSide");
        auto field = LastWorld::get_field();
        double top = field.field_width / 2,
               right = field.field_length / 2 + marginOutsideField,
               bottom = field.field_width / -2,
               left = 0
               ;

        // @Hack omfg this is so f#$%& ugly
        bool isAllWhitespace = std::all_of(
                side.cbegin(),
                side.cend(),
                static_cast<int (*)(int)>(std::isspace)
                );

        if (!(side.empty() || isAllWhitespace || side == "TOTALLY EMPTY")) {
            if (side == "ourSide") {
                left = field.field_length / -2 - marginOutsideField;
                right = 0;
            } else if (side == "theirSide") {
                left = 0;
                right = field.field_length / 2 + marginOutsideField;
            }

            if (newTargetPos.y > top) {
                newTargetPos.y = top;
            }
            if (newTargetPos.x > right) {
                newTargetPos.x = right;
            } 
            if (newTargetPos.y < bottom) {
                newTargetPos.y = bottom;
            }
            if (newTargetPos.x < left) {
                newTargetPos.x = left;
            }
        }
    }
    return newTargetPos;
}


Vector2 GoToPos::limitVel(Vector2 sumOfForces, Vector2 posError) {
    // Limit the robot velocity to the maximum speed
    if (sumOfForces.length() > maxSpeed) {
        if (sumOfForces.length() > 0.0) {
            sumOfForces = sumOfForces.scale(maxSpeed / sumOfForces.length());
        }
    }

    if (minSpeedX > 0 || minSpeedY > 0) {
        double absDrivingAngle = Vector2(fabs(sumOfForces.x), fabs(sumOfForces.y)).angle(); // number between zero and 0.5*pi
        double minSpeed = minSpeedX + ((minSpeedY-minSpeedX) * absDrivingAngle / (0.5*M_PI));

        // If speed is decreasing, going below the minSpeed is allowed because the motors can handle it.
        if (sumOfForces.length() < (minSpeed / 8.0)) {
            if (sumOfForces.length() >= prevSumOfForces.length()) {
                sumOfForces = Vector2(0.0, 0.0);
            }
        } else if (sumOfForces.length() <  minSpeed) {
            if (sumOfForces.length() > 0) {
                sumOfForces = sumOfForces.scale(1/sumOfForces.length() * minSpeed);
            } else {
                sumOfForces = Vector2(0.0, 0.0);
            }
        }
    }    

    prevSumOfForces = sumOfForces;
    return sumOfForces;
}


double GoToPos::limitAngularVel(double angularVelTarget) {
    // If angularVel is decreasing, going below the minAngularVel is allowed because the motors can handle it.
    if (fabs(angularVelTarget) < (minAngularVel / 4)) {
        if (fabs(angularVelTarget) >= fabs(prevAngularVelTarget)) {
            angularVelTarget = 0.0;
        } 
    } else if (fabs(angularVelTarget) < minAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * minAngularVel;
    }

    return angularVelTarget;
}


boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {
    setPresetControlParams();
    
    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    // Get blackboard info
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    KEEPER_ID = GetInt("KEEPER_ID", 100);

    if (HasDouble("pGainRotation")) {
        pGainRotation = GetDouble("pGainRotation");
    }

    if (HasDouble("pGainPosition")) {
        pGainPosition = GetDouble("pGainPosition");
    }

    // if (HasDouble("maxSpeed")) {
    //     maxSpeed = GetDouble("maxSpeed");
    // }

    // if (HasDouble("avoidRobotsGain")) {
    //     avoidRobotsGain = GetDouble("avoidRobotsGain");
    // } else {
        avoidRobotsGain = 0.2;
    // }

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << ROBOT_ID);
        failure = true;
        succeeded = false;
        return boost::none;
    }  

    // Check the input position
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
        prevTargetPos = targetPos;
    }

    // Draw the target position in RQT-view
    drawer.setColor(0, 0, 0);
    drawer.drawPoint("targetPos_" + std::to_string(ROBOT_ID), targetPos);

    // Store some variables for easy access
    Vector2 myPos(me.pos);

    myPosTopic.publish(me);
    
    Vector2 myVel(me.vel);
    Vector2 posError = targetPos - myPos;
    if (HasDouble("angleGoal")) {
        angleGoal = cleanAngle(GetDouble("angleGoal"));
    } else {
        if (posError.length() > 0.5) {
            angleGoal = posError.angle();
        } else {
            angleGoal = me.angle;
        }
    }

    roboteam_msgs::WorldRobot targetPosPub;
    targetPosPub.pos.x = targetPos.x;
    targetPosPub.pos.y = targetPos.y;
    targetPosPub.w = angleGoal;
    myTargetPosTopic.publish(targetPosPub);


    

    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;
    
    double successDist;
    if (HasDouble("successDist")) {
        successDist = GetDouble("successDist");
    } else {
        successDist = 0.02;
    }


    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < successDist && fabs(angleError) < 0.1) {
        sendStopCommand(ROBOT_ID);
        succeeded = true;
        failure = false;
        roboteam_msgs::RobotCommand command;
        return boost::none;
    }

    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    Vector2 sumOfForces(0.0, 0.0);

    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + positionController(myPos, targetPos);

    // Robot avoidance
    if (HasBool("avoidRobots") && GetBool("avoidRobots")) {
        sumOfForces = sumOfForces + avoidRobots(myPos, myVel, targetPos);
        if (posError.length() > 0.5) {
            if (sumOfForces.length() < maxSpeed) {
                sumOfForces = sumOfForces.scale(maxSpeed / sumOfForces.length());
            }
        }
    }


    // Defense area avoidance
    if (HasBool("avoidDefenseAreas") && GetBool("avoidDefenseAreas")) {
        sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);
    } 

    // Ball avoidance
    // if (HasBool("avoidBall") && GetBool("avoidBall")) {
    //     Vector2 ballPos = Vector2(world.ball.pos);
    //     sumOfForces = avoidBall(ballPos, myPos, sumOfForces);
    // }

    
    // Rotation controller to make sure the robot reaches its angleGoal
    double angularVelTarget = rotationController(myAngle, angleGoal, posError);

    
    drawer.setColor(255, 0, 0);
    drawer.drawLine("velCommand" + std::to_string(ROBOT_ID), myPos, sumOfForces);
    drawer.setColor(0, 0, 0);


    // Rotate the commands from world frame to robot frame 
    Vector2 velCommand = worldToRobotFrame(sumOfForces, myAngle);
    
    // Set a minimum and maximum speed for x and y
    velCommand = limitVel(velCommand, posError);
    if (sumOfForces.length() < 0.05) {
        angularVelTarget = limitAngularVel(angularVelTarget);
    }

    if (velCommand.x == 0.0 && velCommand.y == 0.0 && angularVelTarget == 0.0) {
        ROS_INFO_STREAM("command is empty!");
        failure = true;
        success = false;
        return boost::none;
    }

    if (GetBool("smoothDriving")) {
        if ((velCommand.length() - prevVelCommand.length()) > GetDouble("smoothingNumber")) {
            ROS_INFO_STREAM("limiting acceleration!");
            velCommand = velCommand.scale((prevVelCommand.length() + 0.1) / velCommand.length());
        }
        if ((fabs(angularVelTarget) - fabs(prevAngularVelTarget)) > 0.5) {
            ROS_INFO_STREAM("limiting angular acc " << angularVelTarget);
            angularVelTarget = prevAngularVelTarget + (0.5 * signum(angularVelTarget));
            ROS_INFO_STREAM("new angular vel: " << angularVelTarget);
        }
        prevVelCommand = velCommand;
    }
    prevAngularVelTarget = angularVelTarget;

    // Fill the command message
    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = velCommand.x;
    command.y_vel = velCommand.y;
    command.w = angularVelTarget;
    command.dribbler = GetBool("dribbler");
    return command;
}

void GoToPos::Initialize() {
	setPresetControlParams();
}

bt::Node::Status GoToPos::Update() {

    boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
    if (command) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(*command);
        return Status::Running;
    } else {
        if (succeeded) {
            return Status::Running;
        } else if (failure) {
            return Status::Running;
        } else {
            return Status::Invalid;
        }
    }
}

} // rtt