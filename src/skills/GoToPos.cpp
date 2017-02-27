#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>
#include <vector>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Cone.h"

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        
        // Rest of the members
        , success(false)
        // TODO: @Bug Cannot ask potentially private blackboard because private bb is initialized later than the constructor!
        // , maxSpeed(GetDouble("maxSpeed"))
        , attractiveForce(10.0)
        , attractiveForceWhenClose(2.0)
        , repulsiveForce(20.0)
        {print_blackboard(blackboard);}


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
roboteam_utils::Vector2 GoToPos::positionController(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos) {
    roboteam_utils::Vector2 posError = targetPos - myPos;
    roboteam_utils::Vector2 velTarget = posError*attractiveForceWhenClose;
    return velTarget;
}


// Proportional rotation controller
double GoToPos::rotationController(double myAngle, double angleGoal, roboteam_utils::Vector2 posError) {
    // TODO: find out where this is for?
    //if (posError.length() > 0.1) {
    //    angleGoal = posError.angle();
    //}

    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);


    // double timeStep = 1.0/30.0;
    // rotationControllerI += angleError * timeStep;
    // rotationControllerI = 0.9*rotationControllerI + angleError * timeStep;
    // ROS_INFO_STREAM("angleError: " << angleError << ", I effect: " << (rotationControllerI * iGainRotation) << ", P effect: " << (angleError * pGainRotation));

    double angularVelTarget = angleError * pGainRotation;    
    return angularVelTarget;
}


// Integral velocity controller
roboteam_utils::Vector2 GoToPos::velocityController(roboteam_utils::Vector2 velTarget) {
    roboteam_msgs::World world = LastWorld::get();
    roboteam_utils::Vector2 myVel = world.us.at(ROBOT_ID).vel;

    roboteam_utils::Vector2 velError = velTarget - myVel;

    double updateRate;
    ros::param::get("role_iterations_per_second", updateRate);
    double timeStep = 1 / updateRate;

    // TODO: make prettier
    if (velError.length() < 0.3) { 
        velControllerI = velControllerI + velError.scale(timeStep);
    } else { 
        // Velocity difference is too big, probably the robot is still accelerating, in which case it is better
        // to not increase the velControllerI yet... And otherwise there is too much offset in the feedfoward
        // controller anyway. But we can probably think of a better way to detect this...
    }

    // Limit the influence of the I controller
    if (velControllerI.length() > 1.0) {
        velControllerI = velControllerI.scale(1.0 / velControllerI.length());
    }

    roboteam_utils::Vector2 velCommand = velTarget + velControllerI * iGainVelocity;
    return velCommand;
}


// Integral angular velocity controller
double GoToPos::angularVelController(double angularVelTarget) {
    roboteam_msgs::World world = LastWorld::get();
    double myAngularVel = world.us.at(ROBOT_ID).w;

    double angularVelError = angularVelTarget - myAngularVel;

    double updateRate;
    ros::param::get("role_iterations_per_second", updateRate);
    double timeStep = 1 / updateRate;
    angularVelControllerI += angularVelError * timeStep;

    double angularVelCommand = angularVelTarget + angularVelControllerI * iGainAngularVel;
    return angularVelCommand;
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them
roboteam_utils::Vector2 GoToPos::getForceVectorFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, double lookingDistance, Cone antennaCone) {
    roboteam_utils::Vector2 antenna = antennaCone.center - myPos;    

    roboteam_utils::Vector2 forceVector(0.0, 0.0);
    if ((otherRobotPos-myPos).length() < lookingDistance && antennaCone.IsWithinCone(otherRobotPos)) {
        // double distanceToCenter = (otherRobotPos - antenna.closestPointOnVector(myPos, otherRobotPos)).length();
        if (isBetweenAngles(antenna.angle(), antennaCone.side1.angle(), (otherRobotPos - antennaCone.start).angle())) {
            forceVector = antenna.rotate(-0.5*M_PI).scale(2 / (otherRobotPos - myPos).length());
        }
        if (isBetweenAngles(antennaCone.side2.angle(), antenna.angle(), (otherRobotPos - antennaCone.start).angle())) {
            forceVector = antenna.rotate(0.5*M_PI).scale(2 / (otherRobotPos - myPos).length());
        }
    }    
    return forceVector;
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them
double GoToPos::getAngleFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, double lookingDistance, Cone antennaCone) {
    roboteam_utils::Vector2 antenna = antennaCone.center - myPos;    

    double angle;
    if ((otherRobotPos-myPos).length() < lookingDistance && antennaCone.IsWithinCone(otherRobotPos)) {
        // double distanceToCenter = (otherRobotPos - antenna.closestPointOnVector(myPos, otherRobotPos)).length();
        if (isBetweenAngles(antenna.angle(), antennaCone.side1.angle(), (otherRobotPos - antennaCone.start).angle())) {
            angle = antenna.angle()-0.1*M_PI;
        }
        if (isBetweenAngles(antennaCone.side2.angle(), antenna.angle(), (otherRobotPos - antennaCone.start).angle())) {
            angle = antenna.angle()+0.1*M_PI;
        }
    }    
    return angle;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
roboteam_utils::Vector2 GoToPos::avoidRobots(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos) {
    roboteam_msgs::World world = LastWorld::get();

    roboteam_utils::Vector2 posError = targetPos - myPos;
    double lookingDistance = 0.7; // default
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }
    
    roboteam_utils::Vector2 antenna = roboteam_utils::Vector2(lookingDistance, 0.0).rotate(posError.angle());
    roboteam_utils::Vector2 coneStart = myPos - antenna;
    Cone antennaCone(coneStart, (antenna + myPos), 0.3);

    // Draw the lines of the cone in rqt_view
    roboteam_utils::Vector2 coneSide1 = (antennaCone.center-antennaCone.start).rotate(0.5*antennaCone.angle);
    roboteam_utils::Vector2 coneSide2 = (antennaCone.center-antennaCone.start).rotate(-0.5*antennaCone.angle);
    drawer.DrawLine("coneRobotsSide1", antennaCone.start, coneSide1);
    drawer.DrawLine("coneRobotsSide2", antennaCone.start, coneSide2);

    roboteam_utils::Vector2 sumOfForces;
    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (currentRobot.id != ROBOT_ID) {

            roboteam_utils::Vector2 otherRobotPos(currentRobot.pos);
            roboteam_utils::Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
            sumOfForces = sumOfForces + forceVector;
        }
    }

    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 otherRobotPos(world.them.at(i).pos);
        roboteam_utils::Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
        sumOfForces = sumOfForces + forceVector; 
    }

    return sumOfForces;
}

// Computes an angle for the robot to avoid other robots, asumes robot will ride forward
double GoToPos::avoidRobotsForward(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos){

    roboteam_msgs::World world = LastWorld::get();

    roboteam_utils::Vector2 posError = targetPos - myPos;
    double lookingDistance = 0.7; // default
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }
    
    roboteam_utils::Vector2 antenna = roboteam_utils::Vector2(lookingDistance, 0.0).rotate(posError.angle());
    roboteam_utils::Vector2 coneStart = myPos - antenna;
    Cone antennaCone(coneStart, (antenna + myPos), 0.3);

    // Draw the lines of the cone in rqt_view
    roboteam_utils::Vector2 coneSide1 = (antennaCone.center-antennaCone.start).rotate(0.5*antennaCone.angle);
    roboteam_utils::Vector2 coneSide2 = (antennaCone.center-antennaCone.start).rotate(-0.5*antennaCone.angle);
    drawer.DrawLine("coneRobotsSide1", antennaCone.start, coneSide1);
    drawer.DrawLine("coneRobotsSide2", antennaCone.start, coneSide2);

    roboteam_utils::Vector2 sumOfForces;
    double sumOfAngles;

    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (currentRobot.id != ROBOT_ID) {

            roboteam_utils::Vector2 otherRobotPos(currentRobot.pos);
            double angle = getAngleFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
            sumOfAngles = sumOfAngles + angle;
        }
    }

    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 otherRobotPos(world.them.at(i).pos);
        double angle = getAngleFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
        sumOfAngles = sumOfAngles + angle; 
    }

    return sumOfAngles;

}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
roboteam_utils::Vector2 GoToPos::avoidDefenseAreas(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos, roboteam_utils::Vector2 sumOfForces) {
    roboteam_utils::Vector2 posError = targetPos - myPos;

    if (ROBOT_ID != KEEPER_ID) {
        roboteam_utils::Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", myPos, 0.0);
        if (fabs(distToOurDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToOurDefenseArea) > 0) {
            if (sumOfForces.dot(distToOurDefenseArea.rotate(0.5*M_PI)) > 0) {
                sumOfForces = distToOurDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            } else {
                sumOfForces = distToOurDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            }
        }
    }

    roboteam_utils::Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", myPos, 0.0);
    if (fabs(distToTheirDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToTheirDefenseArea) > 0) {
        if (sumOfForces.dot(distToTheirDefenseArea.rotate(0.5*M_PI)) > 0) {
            sumOfForces = distToTheirDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        } else {
            sumOfForces = distToTheirDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        }
    }

    return sumOfForces;
}


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field
roboteam_utils::Vector2 GoToPos::checkTargetPos(roboteam_utils::Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    // We should not go outside the field close to the goal areas
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = 0.0; 
    }

    // If the target position is outside of the field + margins, then change the target position to the closest point within this margin
    if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
        xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
    }
    if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
        yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
    }

    roboteam_utils::Vector2 newTargetPos(xGoal, yGoal);

    // If the current robot is not a keeper, we should take into account that it cannot enter the defense area

    if (ROBOT_ID != KEEPER_ID) {
        
        // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea("our defense area", newTargetPos)) {
            roboteam_utils::Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea("their defense area", newTargetPos)) {
            roboteam_utils::Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToTheirDefenseArea;
        }
    }

    return newTargetPos;
}

namespace {

// TODO: @Hack probably needs to be removed once the robots function properly

enum Mode {
    SERIAL,
    GRSIM,
    GAZEBO
} ;

Mode getMode() {
    std::string robot_output_target = "grsim";
    ros::param::getCached("robot_output_target", robot_output_target);
    if (robot_output_target == "grsim") {
        return Mode::GRSIM;
    } else if (robot_output_target == "serial") {
        return Mode::SERIAL;
    } else {
        return Mode::GAZEBO;
    }
}

} // anonymous namespace

// bt::Node::Status GoToPos::Update() {
roboteam_msgs::RobotCommand GoToPos::getVelCommand() {

    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    maxSpeed = GetDouble("maxSpeed");
    // TODO: @Temporary
    maxSpeed = 1;

    // Get blackboard info
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    if (blackboard->HasInt("KEEPER_ID")) {
        KEEPER_ID = blackboard->GetInt("KEEPER_ID");
    } else {
        // ROS_WARN("GoToPos, KEEPER_ID not set");
        KEEPER_ID = 10;
    }
    


    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    angleGoal = cleanAngle(GetDouble("angleGoal"));    
    

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = lookup_bot(ROBOT_ID, true, &world);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << ROBOT_ID);
    }


    // Checking inputs
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
    }


    // Draw the target position in RQT-view
    drawer.SetColor(0, 0, 0);
    drawer.DrawPoint("targetPos", targetPos);


    // Store some variables for easy access
    roboteam_utils::Vector2 myPos(me.pos);
    roboteam_utils::Vector2 myVel(me.vel);
    roboteam_utils::Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;

    // @HACK: Make oh so smooth qualification turns
    if (HasDouble("angleGoal")) {
        if (angleError > 0.2 || fabs(posError.angle()-myAngle) > 0.2 || posError.length() > 0.8) {
            attractiveForceWhenClose = 5.0;
            double endAngleGoal = GetDouble("angleGoal");
            ROS_INFO_STREAM("angleGoal GoToPos: " << endAngleGoal);
            double angleToTargetPos = posError.angle();
            double angleDiff = cleanAngle(endAngleGoal - angleToTargetPos);
            double distanceFromTarget = posError.length()*0.75;
            if (distanceFromTarget > 1.0) distanceFromTarget = 1.0;
            ROS_INFO_STREAM("distanceFromTarget: " << distanceFromTarget);
            roboteam_utils::Vector2 firstStop = roboteam_utils::Vector2(distanceFromTarget, 0.0).rotate(cleanAngle(endAngleGoal+M_PI));
            firstStop = firstStop.rotate(-0.8*angleDiff);
            firstStop = firstStop + targetPos;
            drawer.SetColor(255, 0, 255);
            drawer.DrawPoint("firstStop2", firstStop);
            targetPos = firstStop;
            posError = targetPos - myPos;
            attractiveForceWhenClose = 5.0;
        } else {
            attractiveForceWhenClose = 2.0;
            drawer.RemovePoint("firstStop2");
        }
    }


    // QUALIFICATION HACK!!!!!:
    // For now, always orient towards our targetPos, and only rotate once we get there
    bool driveBackwards = false;
    if (posError.length() > 0.1) {  

        angleGoal = posError.angle();
        angleError = cleanAngle(angleGoal - myAngle);

        // If we are close to the target we can also drive backwards if that's easier
        // if (posError.length() < 1.0) {
        //     if (fabs(cleanAngle(cleanAngle(posError.angle() + M_PI) - myAngle)) < angleError) {
        //         angleGoal = cleanAngle(posError.angle() + M_PI);
        //         angleError = cleanAngle(angleGoal - myAngle);
        //         driveBackwards = true;
        //     }
        // }

    }


    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    roboteam_utils::Vector2 sumOfForces(0.0, 0.0);


    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + positionController(myPos, targetPos);
    ROS_INFO_STREAM("sumOfForces length:" << sumOfForces.length());

    // Robot avoidance
    if (HasBool("avoidRobots")) {
        if (GetBool("avoidRobots")) {
            sumOfForces = sumOfForces + avoidRobots(myPos, myVel, targetPos);
        }
    } else {
        // ROS_WARN("You did not set the boolean avoidRobots in GoToPos");
    }

    // Robot avoidance
    if (HasBool("avoidRobotsForward")) {
        if (GetBool("avoidRobotsForward")) {
            double avoidAngle = avoidRobotsForward(myPos, myVel, targetPos);
            angleGoal=angleGoal+avoidAngle;
        }
    } else {
        // ROS_WARN("You did not set the boolean avoidRobots in GoToPos");
    }



    // Defense area avoidance
    // sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);
    

    // Rotation controller to make sure the robot has and keeps the correct orientation
    double angularVelTarget = rotationController(myAngle, angleGoal, posError);

    std::cout << "angularVelTarget: " << angularVelTarget << std::endl;

    // Limit the robot velocity to the maximum speed, but also ensure that it goes at maximum speed when not yet close to the target. Because 
    // it might the case that an opponent is blocking our robot, and its sumOfForces is therefore low, but since it is far away from the target
    // it should still go at maximum speed and use the sumOfForces vector only for direction.
    if (posError.length() > 1.0 || sumOfForces.length() > maxSpeed) {
        if (sumOfForces.length() > 0) {
            sumOfForces = sumOfForces.scale(1/sumOfForces.length() * maxSpeed);
        } else {
            sumOfForces = roboteam_utils::Vector2(0.0, 0.0);
        }
    }
    


    // Limit the angular velocity target
    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }



    // QUALIFICATION HACK!!!!!:
    // Integral velocity controller (use only if not too close to the target, to prevent overshoot)
    // roboteam_utils::Vector2 velCommand;
    // if (posError.length() < 0.5) {
    //     velCommand = sumOfForces;
    // } else {
    //     velCommand = velocityController(sumOfForces);
    // }
    // For now, only drive forward in combination with an angular velocity
    double driveSpeed;
    // if (fabs(angleError) > (1.0 / maxSpeed)) {
    if (fabs(angleError) > 0.2) {
        // driveSpeed = 1.0 / fabs(angleError);
        driveSpeed = 1 / (fabs(angleError)*fabs(angleError)) * maxSpeed;
        // driveSpeed = 0;
    } else {
        driveSpeed = maxSpeed;
    }

    if (sumOfForces.length() < driveSpeed) {
        driveSpeed = sumOfForces.length();
    }

    roboteam_utils::Vector2 velCommand;

    // TODO: @Hack Maybe remove this at some point
    auto mode = getMode();

    if (mode == Mode::GRSIM) {
        
        if (driveBackwards) {
            velCommand = roboteam_utils::Vector2(-1.0, 0.0).scale(driveSpeed);
        } else {
            velCommand = roboteam_utils::Vector2(1.0, 0.0).scale(driveSpeed);
        }
    } else {
        if (driveBackwards) {
            velCommand = roboteam_utils::Vector2(0.0, -1.0).scale(driveSpeed);
        } else {
            velCommand = roboteam_utils::Vector2(0.0, 1.0).scale(driveSpeed);
        }
        
    }

    // Integral angular velocity controller
    // TODO: there is not yet an estimation of angular velocities in our world, so this cannot be used yet
    // double angularVelCommand = angularVelController(angularVelTarget);
    // double angularVelCommand = angularVelTarget + (-14 / 512.0 * 2 * M_PI);
    double angularVelCommand = angularVelTarget;


    // QUALIFICATION HACK!!!!!:
    // Rotate the commands from world frame to robot frame 
    // velCommand = worldToRobotFrame(velCommand, myAngle);
    // Draw the velocity vector acting on the robots
    // drawer.DrawLine("velCommand", myPos, velCommand);  

    // Fill the command message
    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = velCommand.x;
    command.y_vel = velCommand.y;
    command.w = angularVelCommand;
    command.dribbler = GetBool("dribbler");

    std::cout << "command.w == " << command.w << "\n";

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    // pub.publish(command);
    // return Status::Running;
    return command;
}

bt::Node::Status GoToPos::Update() {

    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();
    if (world.us.size() == 0) {
        ROS_INFO("No information about the world state :(");
        return Status::Running;
    }


    // Find the robot with the specified ID
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    boost::optional<roboteam_msgs::WorldRobot> findBot = lookup_bot(ROBOT_ID, true, &world);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << ROBOT_ID);
    }


    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    angleGoal = cleanAngle(GetDouble("angleGoal"));


    // Store some variables for easy access
    roboteam_utils::Vector2 myPos(me.pos);
    roboteam_utils::Vector2 myVel(me.vel);
    roboteam_utils::Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;


    // If we are close enough to our target position and target orientation, then stop the robot and return success
    // if (posError.length() < 0.01 && fabs(angleError) < 0.1) {
    //     sendStopCommand(ROBOT_ID);
    //     return Status::Success;
    // }

    // QUALICATION HACK!!!!:

    if (posError.length() < 0.05 && fabs(angleError) < 0.1) {
        sendStopCommand(ROBOT_ID);
        return Status::Success;
    }


    roboteam_msgs::RobotCommand command = getVelCommand();
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
    return Status::Running;
}

} // rtt
