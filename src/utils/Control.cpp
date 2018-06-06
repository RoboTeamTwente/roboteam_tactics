#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/Control.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"


namespace rtt {

Control::Control() : updateRateParam("role_iterations_per_second")
        {
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);
            angleErrorI = 0.0;
            posErrorI = Vector2(0.0, 0.0);

            // To be safe, initialize parameters to zero by default
            pGainPosition = 0.0;
            iGainPosition = 0.0; 
            dGainPosition = 0.0; 
            pGainRotation = 0.0; 
            iGainRotation = 0.0;
            dGainRotation = 0.0;
            pGainVelocity = 0.0;
            maxSpeed = 0.0;
            maxAngularVel = 0.0;
        }

RobotType Control::getRobotType() {

    std::string const robotTypeKey = "robot" + std::to_string(ROBOT_ID) + "/robotType";
    if (ros::param::has(robotTypeKey)) {
        std::string robotTypeString;
        ros::param::get(robotTypeKey, robotTypeString);

        if (robotTypeString == "arduino") {
            return RobotType::ARDUINO;
        } else if (robotTypeString == "proto") {
            return RobotType::PROTO;
        } else if (robotTypeString == "grsim") {
            return RobotType::GRSIM;
        } else if (robotTypeString == "") {
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
                        << robotTypeString
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

void Control::setPresetControlParams(RobotType newRobotType) {
    if (newRobotType == RobotType::ARDUINO) {
        pGainPosition = 1.0;
        iGainPosition = 0.0;
        pGainRotation = 2.0;
        iGainRotation = 0.0;
        maxSpeed = 3.0; 
        maxAngularVel = 10.0;
        dGainPosition = 0.0;
        
        thresholdTarget = 0.1;
        minTarget = 0.30;

        robotType = RobotType::ARDUINO;
    } else if (newRobotType == RobotType::PROTO) {
        pGainPosition = 4.5;//3.0
        iGainPosition = 0.0;//prevteam: 0.0 //kantoor:0.3 //DL: 0.2
        dGainPosition = 0.8; //prevteam: 0.5
        pGainRotation = 10;
        iGainRotation = 0.0;//prevteam: 0.0
        dGainRotation = 0.0;
        pGainVelocity = 0.0;
        maxSpeed = 5.0;
        maxAngularVel = 20.0;

        thresholdTarget = 0.05;
        minTarget = 0.60;

        robotType = RobotType::PROTO;
    } else if (newRobotType == RobotType::GRSIM) {
        pGainPosition = 2.0; //2.0
        iGainPosition = 0.0;
        dGainPosition = 0.0;
        pGainRotation = 2.0;
        iGainRotation = 0.0;
        maxSpeed = 2.0;
        maxAngularVel = 10.0;

        thresholdTarget = 0.03;
        minTarget = 0.10;

        robotType = RobotType::GRSIM;
    } else {
        if (time_difference_milliseconds(lastPresetError, now()).count() >= 1000) {
            ROS_ERROR_STREAM("Could not set robot type of " 
                    << ROBOT_ID
                    << ". Undefined type. Leaving the type on: "
                    << (int) robotType
                    << "\n"
                    );

            lastPresetError = now();
        }
    }
}

void Control::setPresetControlParams() {
    setPresetControlParams(robotType);
}

void Control::setPresetControlParams(
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
	this->maxSpeed = maxSpeed;
}

void Control::setControlParam(std::string paramName, double paramValue) {
    if (paramName == "pGainPosition") {
        this->pGainPosition = paramValue;
    }
    else if (paramName == "iGainPosition") {
        this->iGainPosition = paramValue;
    }
    else if (paramName == "dGainPosition") {
        this->dGainPosition = paramValue;
    }
    else if (paramName == "pGainRotation") {
        this->pGainRotation = paramValue;
    }
    else if (paramName == "iGainRotation") {
        this->iGainRotation = paramValue;
    }
    else if (paramName == "dGainRotation") {
        this->dGainRotation = paramValue;
    }
    else if (paramName == "maxSpeed") {
        this->maxSpeed = paramValue;
    }
    else if (paramName == "maxAngularVel") {
        this->maxAngularVel = paramValue;
    }
    else if (paramName == "thresholdTarget") {
        this->thresholdTarget = thresholdTarget;
    }
    else if (paramName == "minTarget") {
        this->minTarget = minTarget;
    }
}


roboteam_msgs::RobotCommand Control::getStopCommand(uint id) {
    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    command.dribbler = false;
    return command;
}

roboteam_msgs::RobotCommand Control::getStopCommand(uint id, bool dribbler) {
    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    command.dribbler = dribbler;
    return command;
}


// PI position controller
Vector2 Control::positionController(Vector2 myPos, Vector2 targetPos) {

    Vector2 posError = targetPos - myPos;

    // Integral term
    if (posError.length() < 0.15) {
        posErrorI = posErrorI.scale(0.95) + posError.scale(1);
    } else {
        posErrorI = Vector2(0.0, 0.0);
    }

    // Control equation
    Vector2 velTarget = posError*pGainPosition + posErrorI*iGainPosition;
    
    // Limit velocity target to the maxSpeed
    if (velTarget.length() > maxSpeed) {
        velTarget = velTarget.scale(maxSpeed / velTarget.length());
    }

    return velTarget;
}

// PID position controller
Vector2 Control::positionController(Vector2 myPos, Vector2 targetPos, Vector2 myVel) {
 
    Vector2 posError = targetPos - myPos;

    // Integral term (SEEMS HACKY, ALSO IT SHOULD TAKE TIME STEP INTO ACCOUNT)
    if (posError.length() < 0.20 && posError.length() > 0.04) {
        posErrorI = posErrorI.scale(0.95) + posError.scale(1);
    } else {
        posErrorI = Vector2(0.0, 0.0);
    }

    if (myVel.length() < 0.05) { // myVel becomes unreliable at too low value
        myVel = Vector2(0.0,0.0);
    }
    // Control equation
    Vector2 velTarget = posError*pGainPosition + posErrorI*iGainPosition - myVel*dGainPosition; // should use derivative of error instead of myVel

    // Limit velocity target to the maxSpeed
    if (velTarget.length() > maxSpeed) {
        velTarget = velTarget.scale(maxSpeed / velTarget.length());
    }

    // If velocity target is too low, it's set to a minimum value
    if (velTarget.length() < minTarget) {
        if (velTarget.length() > thresholdTarget) {
            velTarget = velTarget.scale(minTarget / velTarget.length());  
        }
    }   

    return velTarget;
}


// Proportional velocity controller NOT USED ANYMORE
Vector2 Control::velocityController(Vector2 myVelRobotFrame, Vector2 velTarget) {

    Vector2 velError = prevVelCommand - myVelRobotFrame;
    Vector2 velControl = velError * pGainVelocity;
    Vector2 newCommand = velTarget + velControl;
    prevVelCommand = velTarget;

    return newCommand;
}

// Proportional rotation controller
double Control::rotationController(double myAngle, double angleGoal, Vector2 posError) {

    bool forceAngle = false;


    if (posError.length() > 1.0 && !forceAngle) {
        angleGoal = posError.angle();
    }

    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);
    // ROS_INFO_STREAM("targetAngle: " << angleGoal << " myAngle: " << myAngle << " angleError: " << angleError);

    // Integral term
    double updateRate = updateRateParam();
    if (!updateRateParam.isSet() || updateRate == 0) {
        if (updateRate == 0) {
            ROS_WARN_STREAM_THROTTLE(1, "role_iterations_per_second set to 0! changing to 30.");
        } else {
            ROS_WARN_STREAM_THROTTLE(1, "role_iterations_per_second not set! Assuming 30.");
        }
        updateRate = 30;
    }

    double timeStep = 1 / updateRate;
    // if (fabs(angleError) < 0.15*M_PI) {
        angleErrorI = angleErrorI*0.9 + angleError * timeStep;
    // } else {
        // angleErrorI = 0;
    // }

    // Control equation
    double angularVelTarget = angleError * pGainRotation + angleErrorI * iGainRotation;

    // Limit the angular velocity target
    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }


    return angularVelTarget;
}



// Proportional rotation controller
double Control::rotationController(double myAngle, double angleGoal, Vector2 posError, double myAngularVel) {

    bool forceAngle = false;

    if (posError.length() > 1.0 && !forceAngle) {
        angleGoal = posError.angle();
    }
    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);
    // ROS_INFO_STREAM("targetAngle: " << angleGoal << " myAngle: " << myAngle << " angleError: " << angleError);

    // Integral term
    double updateRate = updateRateParam();
    if (!updateRateParam.isSet() || updateRate == 0) {
        if (updateRate == 0) {
            ROS_WARN_STREAM_THROTTLE(1, "role_iterations_per_second set to 0! changing to 30.");
        } else {
            ROS_WARN_STREAM_THROTTLE(1, "role_iterations_per_second not set! Assuming 30.");
        }
        updateRate = 30;
    }

    double timeStep = 1 / updateRate;
    // if (fabs(angleError) < 0.15*M_PI) {
        angleErrorI = angleErrorI + angleError * timeStep;
    // } else {
        // angleErrorI = 0;
    // }

    if (fabs(angleError) < 0.1*M_PI) {
        angleErrorI = 0.0;
    }

    // Control equation
    double angularVelTarget = angleError * pGainRotation + angleErrorI * iGainRotation - myAngularVel * dGainRotation;

    if (fabs(angularVelTarget) < 2.2) {
        if (fabs(angularVelTarget) > 0.5) {
            angularVelTarget = angularVelTarget / fabs(angularVelTarget) * 2.2;
        }
    }

    return angularVelTarget;
}


Vector2 Control::limitVel2(Vector2 sumOfForces, double angleError) {
    double limit = fmin(maxSpeed, maxSpeed/fabs(angleError*2));
    double L = sumOfForces.length();
    // Limit the robot velocity to the maximum speed
    if (L > limit && L > 0.0) {
        sumOfForces = sumOfForces.scale(limit / L);
    }
    return sumOfForces;
}

Vector2 Control::limitVel(Vector2 sumOfForces, double angularVelTarget) {

    double L = sumOfForces.length();
    if (angularVelTarget >= 7.0) {
        // Limit the robot velocity to lower speed
        if (L > 0.5) {
            sumOfForces = sumOfForces.scale(0.5 / L);
        }
    } else {
    // Limit the robot velocity to the maximum speed
        if (L > maxSpeed && L > 0.0) {
            sumOfForces = sumOfForces.scale(maxSpeed / L);
        }
    }
        // WHY IS THIS HERE?
    if (fabs(sumOfForces.y) > 1.0) {
        sumOfForces = sumOfForces.scale(1.0 / fabs(sumOfForces.y));
    }

    return sumOfForces;
}


double Control::limitAngularVel(double angularVelTarget) {

    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }

    return angularVelTarget;
}


void Control::Initialize(int ROBOT_ID) {
    this->ROBOT_ID = ROBOT_ID;
    robotType = getRobotType(); // Set robotType for global use
    lastRobotTypeError = now();
    lastPresetError = now();
	setPresetControlParams();
    return;
}

} // rtt
