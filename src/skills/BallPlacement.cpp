#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/BallPlacement.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"

#define RTT_CURRENT_DEBUG_TAG BallPlacement

namespace rtt {


    RTT_REGISTER_SKILL(BallPlacement);

    BallPlacement::BallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(name, blackboard)
            , rotateAroundPoint("", private_bb)
    {
        succeeded = false;
        failure = false;
        ros::NodeHandle n;
        myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
        myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
        myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);
        controller.Initialize(blackboard->GetInt("ROBOT_ID"));

        safetyMarginGoalAreas = 0.2;
        marginOutsideField = 0.3;
        avoidRobotsGain = 0.15;

        ROS_INFO("BallPlacement");
    }

// gives stop command
    void BallPlacement::stoprobot(int robotID) {
        std::cout << "Stopping robot!\n";

        rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(stop_command(robotID));
    }

// returns robotRequiredv vector??
    Vector2 BallPlacement::worldToRobotFrame(Vector2 requiredv, double rotation){
        Vector2 robotRequiredv;
        robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
        robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
        return robotRequiredv;
    }

// returns the difference angle between robotpos and faceTowardsPos (angle error??)
    double BallPlacement::computeAngle(Vector2 robotPos, Vector2 faceTowardsPos) {
        Vector2 differenceVector = faceTowardsPos - robotPos;
        return differenceVector.angle();
    }

// cleans the angle??
    double BallPlacement::cleanAngle(double angle){
        if (angle < M_PI){
            return fmod(angle-M_PI, (2*M_PI))+M_PI;
        }
        else if(angle > M_PI){
            return fmod(angle+M_PI, (2*M_PI))-M_PI;
        }
        else {
            return angle;
        }
    }

// slows down required velocity vector??
    Vector2 BallPlacement::saveDribbleDeceleration(Vector2 reqspeed){
        // TODO: should improve timing accuracy, because rate is not really known.
        Vector2 deceleration=prevspeed-reqspeed;
        double timestep=1.0/60;
        double maxdeceleration=1.0;
        Vector2 speed;
        if(deceleration.length() > timestep*maxdeceleration){
            speed=prevspeed-deceleration.normalize()*maxdeceleration*timestep;
        }
        else {
            speed=reqspeed;
        }

        prevspeed=speed;

        return speed;
    }



// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them7
// Based on this algorithm: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
    Vector2 BallPlacement::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos) {

        Vector2 forceVector;

        Vector2 ahead = myPos + antenna;
        Vector2 closestPoint = antenna.closestPointOnVector(myPos, otherRobotPos);

        double dist = (closestPoint - otherRobotPos).length();

        if (closestPoint != myPos && closestPoint != ahead && dist <= 0.5) {
            Vector2 force = closestPoint - otherRobotPos;
            force = force.scale(avoidRobotsGain / (force.length() * force.length()) );
            force = force.scale(1.0 / (closestPoint - myPos).length());
            forceVector = force;
        }

        return forceVector;
    }


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
    Vector2 BallPlacement::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
        roboteam_msgs::World world = LastWorld::get();

        Vector2 posError = targetPos - myPos;
        double lookingDistance = 1.0; // default
        if (lookingDistance > (posError.length())) {
            lookingDistance = posError.length();
        }

        // The antenna is a vector starting at the robot position in the direction in which it is driving (scaled to the robot speed)
        Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
        antenna = antenna.scale(myVel.length()*1.0); // magic scaling constant


        // For all robots in the field that are closer than the lookingDistance to our robot, determine if they exert a repelling force and add all these forces
        Vector2 sumOfForces;
        for (auto const currentRobot : world.us) {
            if (currentRobot.id != ROBOT_ID) {
                Vector2 otherRobotPos(currentRobot.pos);
                Vector2 otherRobotVel(currentRobot.vel);
                double distToRobot = (otherRobotPos - myPos).length();
                Vector2 otherRobotFuturePos = otherRobotPos + otherRobotVel.scale(distToRobot / myVel.length());
                if ((otherRobotFuturePos - myPos).length() <= lookingDistance) {
                    std::cout << "HELP A ROBOT OF OUR TEAM" << std::endl;

                    Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotFuturePos, antenna, targetPos);
                    sumOfForces = sumOfForces + forceVector;
                }
            }
        }
        for (size_t i = 0; i < world.them.size(); i++) {
            Vector2 otherRobotPos(world.them.at(i).pos);
            Vector2 otherRobotVel(world.them.at(i).vel);
            double distToRobot = (otherRobotPos - myPos).length();
            Vector2 otherRobotFuturePos = otherRobotPos + otherRobotVel.scale(distToRobot / myVel.length());
            if ((otherRobotFuturePos - myPos).length() <= lookingDistance) {
                std::cout << "HELP A ROBOT OF THEIR TEAM" << std::endl;
                Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotFuturePos, antenna, targetPos);
                sumOfForces = sumOfForces + forceVector;
            }
        }

        drawer.setColor(255, 0, 0);
        drawer.drawLine("sumOfForces", myPos, sumOfForces);
        drawer.setColor(0, 0, 0);

        return sumOfForces;
    }


// dribble and not GoToPos
    bt::Node::Status BallPlacement::Update() {
        roboteam_msgs::World world = LastWorld::get();
        int robotID = blackboard->GetInt("ROBOT_ID");

        if (!LastWorld::have_received_first_world()) {
            return Status::Running;
        }

        if (world.header.seq == prevSeq) {
            return Status::Running;
        } else {
            prevSeq = world.header.seq;
        }

        roboteam_msgs::WorldBall ball = world.ball;

        // Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update
        // is called before the first world state update
        if (world.us.size() == 0) {
            ROS_INFO("No information about the world state :(");
            return Status::Running;
        }

        roboteam_msgs::WorldRobot robot;
        if (auto robotOpt = getWorldBot(robotID, true)) {
            robot = *robotOpt;
        } else {
            ROS_ERROR_STREAM("[BallPlacement] Could not find robot #" << robotID << " in world! Failing.");
            return Status::Failure;
        }

        Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
        Vector2 robotVel(robot.vel);
        Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
        Vector2 goalPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
        //position error GoToPos
        Vector2 posError = goalPos - ballPos;
        //Position error dribble
        Vector2 goalposDiff = ballPos - goalPos;

        double targetAngle = computeAngle(robotPos, goalPos);

        double myAngle = robot.angle;
        // double angleError = cleanAngle(targetAngle - myAngle);

        double worldrotDiff = (robotPos - ballPos).angle() - (targetAngle + M_PI);
        worldrotDiff = cleanAngle(worldrotDiff);
        double worldrottoballdiff = cleanAngle(goalposDiff.angle() - myAngle+ M_PI);

        if(goalposDiff.length() > 0.02){
            if((robotPos-ballPos).length() > 0.2){
                // ROS_INFO("lost ball");
                stoprobot(robotID);
                return Status::Failure;
            }

            Vector2 rotatearoundPos;

            if (worldrotDiff > 20.0/180.0*M_PI or worldrotDiff < -20.0/180.0*M_PI){ // oriented towards goal behind ball
                // ROS_INFO("rotate around ball");
                rotatearoundPos = ballPos;
            }
            else {
                // ROS_INFO("rotate around something else");
                rotatearoundPos = goalPos - goalposDiff * (fabs(worldrottoballdiff) / 45.0);
            }

            Vector2 facetowardsPos = goalPos * 2 - ballPos;

            double radius=(rotatearoundPos-robotPos).length();
            // debug points;
            // roboteam_msgs::DebugPoint debugpoint;
            // debugpoint.name = "rotatearound";
            // debugpoint.remove = false;
            // debugpoint.pos.x = rotatearoundPos.x;
            // debugpoint.pos.y = rotatearoundPos.y;
            // debugpoint.color.r = 255;
            // debugpoint.color.g = 0;
            // debugpoint.color.b = 0;
            // pubDebugpoints.publish(debugpoint);


            private_bb->SetInt("ROBOT_ID", robotID);
            private_bb->SetBool("quiet", true);
            private_bb->SetString("center", "point");
            private_bb->SetDouble("centerx", rotatearoundPos.x);
            private_bb->SetDouble("centery", rotatearoundPos.y);
            private_bb->SetDouble("radius", radius);
            private_bb->SetDouble("faceTowardsPosx", facetowardsPos.x);
            private_bb->SetDouble("faceTowardsPosy", facetowardsPos.y);
            private_bb->SetDouble("w", 3.0);

            double maxv = 0.3;
            // the higher worldrotDiff, the lower maxv

            if (fabs(worldrotDiff) < 45.0 / 180.0 * M_PI) {
                maxv = maxv * (45.0 / 180.0 * M_PI - worldrotDiff) / (45.0 / 180.0 * M_PI);
            } else {
                maxv = 0.0;
            }

            double vPconstant = 1.5;

            Vector2 vtogoal = goalposDiff * -vPconstant;

            // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
            Vector2 sumOfForces;

            // Position controller to steer the robot towards the target position
            sumOfForces = vtogoal;
            std::cout << "1 - position control x: " << sumOfForces.x << ", x: " << sumOfForces.y << std::endl;

            // Robot avoidance
            if (HasBool("avoidRobots") && !GetBool("avoidRobots")) {

                // AvoidRobots defaults to true if not set
                std::cout << "2 - default of avoid robots, x: " << sumOfForces.x << ", y: " << sumOfForces.y << std::endl;

            } else {
                Vector2 newSumOfForces = sumOfForces + avoidRobots(robotPos, robotVel, goalPos);

                std::cout << "3 - difference is x: " << newSumOfForces.x-sumOfForces.x << ", y: " << newSumOfForces.y-sumOfForces.y << std::endl;

                if (posError.length() >= 0.5) {
                    targetAngle = sumOfForces.angle();
                    // angleError = cleanAngle(targetAngle - myAngle);
                }
                sumOfForces = newSumOfForces;

            }

            //hier dingen bij optellen

            // std::cout << "maxv" << maxv << "\n";
            // std::cout << "vtogoal: " << vtogoal << "\n";

            if(sumOfForces.length() > maxv){
                sumOfForces = sumOfForces / sumOfForces.length() * maxv;
            }

            robotvtogoal = worldToRobotFrame(sumOfForces, myAngle);

            // debugpoint.name = "oldrobotvtogoal";
            // debugpoint.remove = false;
            // debugpoint.pos.x = robotvtogoal.x;
            // debugpoint.pos.y = robotvtogoal.y;
            // debugpoint.color.r = 0;
            // debugpoint.color.g = 0;
            // debugpoint.color.b = 255;
            // pubDebugpoints.publish(debugpoint);


            robotvtogoal = saveDribbleDeceleration(robotvtogoal);

            private_bb->SetDouble("extravx", robotvtogoal.x);
            private_bb->SetDouble("extravy", robotvtogoal.y);
            private_bb->SetDouble("maxv", 0.3);

        } else {
            stoprobot(robotID);

            return Status::Success;

        }

        auto rapStatus = rotateAroundPoint.Update();

        if (rapStatus != Status::Running) {
            // ROS_INFO("RAP was not running, so sending our own command!");
            // send command
            roboteam_msgs::RobotCommand cmd;
            cmd.id = robotID;
            cmd.active = true;
            cmd.x_vel = robotvtogoal.x;
            cmd.y_vel = robotvtogoal.y;
            cmd.w = 0;

            cmd.dribbler=true;
            cmd.kicker=false;
            cmd.kicker_vel=0.0;
            cmd.kicker_forced=false;
            cmd.chipper=false;
            // if(extrav.x>0.01 or extrav.y>0.01){
            // cmd.kicker_vel=0.01;
            // }
            cmd.chipper_vel=0.0;
            cmd.chipper_forced=false;
            //ROS_INFO("rotDiff:%f cmd vel x:%f, y:%f, w:%f", worldrotDiff ,cmd.x_vel,cmd.y_vel,cmd.w);
            rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(cmd);
        }

        return Status::Running;
    }

} // rtt
