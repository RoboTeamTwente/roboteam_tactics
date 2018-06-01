#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/SimpleDefender.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG SimpleDefender
#define ROS_LOG_NAME "skills.SimpleDefender"

namespace rtt {

RTT_REGISTER_SKILL(SimpleDefender);


SimpleDefender::SimpleDefender(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb) { }

Vector2 SimpleDefender::computeDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset) {

    Vector2 goalPos;
    if (ourSide) {
        goalPos = LastWorld::get_our_goal_center();
    } else {
        goalPos = LastWorld::get_their_goal_center();
    }

    double angle = (defendPos - goalPos).angle() + angleOffset;

    //if (((defendPos - goalPos).length() - 0.5) < distanceFromGoal) {
        //distanceFromGoal = (defendPos - goalPos).length() - 0.5;
    //}

    Vector2 targetPos(distanceFromGoal, 0.0);
    targetPos = targetPos.rotate(angle);
    targetPos = goalPos + targetPos;

    return targetPos;
}

Vector2 SimpleDefender::computeDefensePointRatio(Vector2 targetFrom, Vector2 targetTo, double ratio){
    // Get the position of our goal
    Vector2 goalPos(LastWorld::get_our_goal_center());
    // Get vector between defendPos and goalPos
    Vector2 targetPos = targetTo - targetFrom;
    // Scale vector to ratio
    targetPos = targetPos.scale(ratio);
    // Move vector to goalPos
    targetPos = targetPos + targetFrom;

    return targetPos;
}

Vector2 SimpleDefender::computeDefensePointAbsolute(Vector2 targetFrom, Vector2 targetTo, double distance){
    // Get the position of our goal
    Vector2 goalPos(LastWorld::get_our_goal_center());
    // Get vector between defendPos and goalPos
    Vector2 targetPos = targetTo - targetFrom;
    // Stretch vector to correct length
    targetPos.stretchToLength(distance);
    // Move vector to goalPos
    targetPos = targetPos + targetFrom;

    return targetPos;
}

Vector2 SimpleDefender::getTargetFromPosition(){
    ROS_INFO_STREAM_NAMED("SimpleDefender", "getTargetFromPosition");
    // for targetFrom, pak de Vector2 van de target
    if(HasString("targetFromType")){
        // Get type
        std::string type = GetString("targetFromType");
        if(type == "position"){
            double x = GetDouble("targetFromTypeX");
            double y = GetDouble("targetFromTypeY");
            ROS_INFO_STREAM_NAMED("SimpleDefender", " [" << x << " ," << y << "]");

            return Vector2(x, y);
        }

        if(type == "object"){
            std::string obj = GetString("targetFromObj");
            if(obj == "them" || obj == "us"){
                int robotID = GetInt("targetFromRobotId");
                if(obj == "them"){
                    return (Vector2)LastWorld::get().them.at(robotID).pos;
                }
                if(obj == "us"){
                    return (Vector2)LastWorld::get().us.at(robotID).pos;
                }
            }
            if(obj == "ball"){
                return (Vector2)LastWorld::get().ball.pos;
            }
        }
    }

    ROS_WARN_STREAM_NAMED("SimpleDefender", "should not be here!");
}

Vector2 SimpleDefender::getTargetToPosition(){
    // for targetFrom, pak de Vector2 van de target
    if(HasString("targetToType")){
        // Get type
        std::string type = GetString("targetToType");
        if(type == "position"){
            double x = GetDouble("targetToTypeX");
            double y = GetDouble("targetToTypeY");
            return Vector2(x, y);
        }

        if(type == "object"){
            std::string obj = GetString("targetToObj");
            if(obj == "them" || obj == "us"){
                int robotID = GetInt("targetToRobotId");
                if(obj == "them"){
                    return (Vector2)LastWorld::get().them.at(robotID).pos;
                }
                if(obj == "us"){
                    return (Vector2)LastWorld::get().us.at(robotID).pos;
                }
            }
            if(obj == "ball"){
                return (Vector2)LastWorld::get().ball.pos;
            }
        }
    }
    ROS_WARN_STREAM_NAMED("SimpleDefender", "should not be here!");
}


bt::Node::Status SimpleDefender::Update() {

    // Get the last world information and some blackboard info
    roboteam_msgs::World world = LastWorld::get();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    robotID = blackboard->GetInt("ROBOT_ID");

    bool ourSide = true;
    if (HasBool("ourSide")) {
        ourSide = GetBool("ourSide");
    }

    double distanceFromGoal = 0.7;
    double distanceFromGoalRatio = 0.5;
    double acceptableDeviation = 0.8;
    double dribblerDist = 2.0;

    if (HasDouble("distanceFromGoal")) {
        distanceFromGoal = GetDouble("distanceFromGoal");
    }
    if (HasDouble("distanceFromGoalRatio")) {
        distanceFromGoalRatio = GetDouble("distanceFromGoalRatio");
    }
    if (HasDouble("acceptableDeviation")) {
        acceptableDeviation = GetDouble("acceptableDeviation");
    }
    if (HasDouble("dribblerDist")) {
        dribblerDist = GetDouble("dribblerDist");
    }


    // DistanceFromGoal
    // DistanceFromGoalAmorousAdventure

    Vector2 defendPos;

    if (HasInt("defendRobot")) {
        ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "parameter 'defendRobot' can be replaced with 'targetTo'");
        unsigned int defendRobot = GetInt("defendRobot");
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(defendRobot, false, world);
        roboteam_msgs::WorldRobot robot;
        if (findBot) {
            robot = *findBot;
        } else {
            ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Robot with this ID not found, ID: " << robotID);
            return Status::Failure;
        }
        defendPos = Vector2(robot.pos);
    } else {
        defendPos = Vector2(world.ball.pos);
    }

    double angleOffset = GetDouble("angleOffset");
    Vector2 targetPos;

    if(HasDouble("distanceFromGoalRatio")) {
        Vector2 targetFromPosition = getTargetFromPosition();
        Vector2 targetToPosition = getTargetToPosition();
        targetPos = computeDefensePointRatio(targetFromPosition, targetToPosition, distanceFromGoalRatio);
    }else
    if(HasDouble("distanceFromGoalAbsolute")){
        Vector2 targetFromPosition = getTargetFromPosition();
        Vector2 targetToPosition = getTargetToPosition();
        targetPos = computeDefensePointAbsolute(targetFromPosition, targetToPosition, distanceFromGoalRatio);
    }else{
        ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Using old computeDefensePoint");
        targetPos = computeDefensePoint(defendPos, ourSide, distanceFromGoal, angleOffset);
    }

    // put targetPos into GoToPos
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    private_bb->SetDouble("receiveBallAtX", targetPos.x);
    private_bb->SetDouble("receiveBallAtY", targetPos.y);
    private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
    private_bb->SetDouble("dribblerDist", dribblerDist);
    private_bb->SetBool("defenderMode", true);
    private_bb->SetBool("dontDriveToBall", GetBool("dontDriveToBall"));
    private_bb->SetBool("setSignal", false);
    if (HasBool("avoidBallsFromOurRobots") && GetBool("avoidBallsFromOurRobots")) {
        private_bb->SetBool("avoidBallsFromOurRobots", true);
    }

    if (HasBool("avoidBall") && GetBool("avoidBall")) {
        private_bb->SetBool("avoidBall", true);
    }

    if (HasString("stayOnSide")) {
        private_bb->SetString("stayOnSide", GetString("stayOnSide"));
    }
    if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        private_bb->SetBool("stayAwayFromBall", true);
    }
    if (HasDouble("maxSpeed")) {
        private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
    }
    if (HasBool("enterDefenseAreas")) {
        private_bb->SetBool("enterDefenseAreas", GetBool("enterDefenseAreas"));
    }
    if (HasBool("pGainLarger")){
            private_bb->SetBool("pGainLarger", GetBool("pGainLarger"));
    }
    if (HasDouble("marginDeviation")) {
		private_bb->SetDouble("marginDeviation", GetDouble("marginDeviation"));
	}


    return receiveBall.Tick();
}

} // rtt
