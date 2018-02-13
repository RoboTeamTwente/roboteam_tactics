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

void Control::setPresetControlParams(RobotType newRobotType) {
    if (newRobotType == RobotType::ARDUINO) {
        pGainPosition = 1.0;
        iGainPosition = 0.0;
        pGainRotation = 2.0;
        iGainRotation = 0.0;
        maxSpeed = 3.0; 
        maxAngularVel = 10.0;
        dGainPosition = 0.0;

        minTarget = 0.07;

        robotType = RobotType::ARDUINO;
    } else if (newRobotType == RobotType::PROTO) {
        pGainPosition = 2.0;//3.0
        iGainPosition = 0.0;//prevteam: 0.0 //kantoor:0.3 //DL: 0.2
        dGainPosition = 0.3; //prevteam: 0.5
        pGainRotation = 2.5; 
        iGainRotation = 0.0;//prevteam: 0.0
        dGainRotation = 0.0;
        pGainVelocity = 0.0;
        maxSpeed = 4.0;
        maxAngularVel = 10.0;

        minTarget = 0.02;

        robotType = RobotType::PROTO;
    } else if (newRobotType == RobotType::GRSIM) {
        pGainPosition = 2.0;//2.0
        iGainPosition = 0.0;//0.0
        dGainPosition = 0.0; //0.0
        pGainRotation = 2.0;
        iGainRotation = 0.0;
        maxSpeed = 2.0;
        maxAngularVel = 10.0;

        minTarget = 0.07;

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
    setPresetControlParams(getRobotType());
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
    if (paramName == "iGainPosition") {
        this->iGainPosition = paramValue;
    }
    if (paramName == "dGainPosition") {
        this->dGainPosition = paramValue;
    }
    if (paramName == "pGainRotation") {
        this->pGainRotation = paramValue;
    }
    if (paramName == "iGainRotation") {
        this->iGainRotation = paramValue;
    }
    if (paramName == "dGainRotation") {
        this->dGainRotation = paramValue;
    }
    if (paramName == "maxSpeed") {
        this->maxSpeed = paramValue;
    }
    if (paramName == "maxAngularVel") {
        this->maxAngularVel = paramValue;
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

    // Integral term
    if (posError.length() < 0.20 && posError.length() > 0.04) {
        posErrorI = posErrorI.scale(0.95) + posError.scale(1);
    } else {
        posErrorI = Vector2(0.0, 0.0);
    }

    // Control equation
    Vector2 velTarget = posError*pGainPosition + posErrorI*iGainPosition - myVel*dGainPosition;

    // Limit velocity target to the maxSpeed
    if (velTarget.length() > maxSpeed) {
        velTarget = velTarget.scale(maxSpeed / velTarget.length());
    }

    if (velTarget.length() < 0.30) {
        if (velTarget.length() > minTarget) {
            velTarget = velTarget.scale(0.30 / velTarget.length());  
        }
    }

    return velTarget;
}


// Proportional velocity controller
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

    // Limit the angular velocity target
    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }

    if (fabs(angularVelTarget) < 2.2) {
        if (fabs(angularVelTarget) > 0.5) {
            angularVelTarget = angularVelTarget / fabs(angularVelTarget) * 2.2;
        }
    }


    return angularVelTarget;
}


Vector2 Control::limitVel(Vector2 sumOfForces, double angularVelTarget) {

    // if (angularVelTarget >= 7.0) {

    //     // Limit the robot velocity to the maximum speed
    //     if (sumOfForces.length() > 1.5) {
    //         if (sumOfForces.length() > 0.0) {
    //             sumOfForces = sumOfForces.scale(1.5 / sumOfForces.length());
    //         }
    //     }

    // } else {

        // Limit the robot velocity to the maximum speed
        if (sumOfForces.length() > maxSpeed) {
            if (sumOfForces.length() > 0.0) {
                sumOfForces = sumOfForces.scale(maxSpeed / sumOfForces.length());
            }
        }

    // }


    

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
    lastRobotTypeError = now();
    lastPresetError = now();
	setPresetControlParams();
    return;
}

} // rtt
