#pragma once

#include "ros/ros.h"

#include <vector>
#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"


namespace rtt {

enum class RobotType {
    ARDUINO,
    PROTO,
    GRSIM
} ;


class Control {
public:
    Control();

    roboteam_msgs::RobotCommand getStopCommand(uint id);
    roboteam_msgs::RobotCommand getStopCommand(uint id, bool dribbler);

    Vector2 positionController(Vector2 myPos, Vector2 targetPos);
    Vector2 velocityController(Vector2 myVelRobotFrame, Vector2 targetVel);
    double rotationController(double myAngle, double angleGoal, Vector2 posError);
    Vector2 velocityController();
    double angularVelController();

    Vector2 limitVel(Vector2 sumOfForces);
    double limitAngularVel(double angularVelTarget);
    void Initialize(int ROBOT_ID);
    
    std::string node_name() { return "Control"; }
    
    RobotType getRobotType();
    void setPresetControlParams(RobotType newRobotType);
    void setPresetControlParams();
    void setPresetControlParams(
    	double pGainPosition,
    	double pGainRotation,
    	double maxAngularVel,
    	double minAngularVel,
    	double maxSpeed,
    	double minSpeedX,
    	double minSpeedY
    );
    // void setPGains(double position, double rotation);
    
private:
    int ROBOT_ID;

    // Control parameters
    double pGainPosition;
    double iGainPosition;
    double pGainRotation;
    double iGainRotation;
    double pGainVelocity;

    // Min and max speeds
    double maxAngularVel;
    double maxSpeed;

    Vector2 prevVelCommand;
    double prevAngularVelTarget;
    
    Vector2 posErrorI;
    double angleErrorI;

    Draw drawer;
    RobotType robotType;

    time_point lastRobotTypeError;
    time_point lastPresetError;

    ros::Publisher myPosTopic;
    ros::Publisher myVelTopic;
    ros::Publisher myTargetPosTopic;
} ;

} // rtt